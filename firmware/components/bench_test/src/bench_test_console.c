#include "bench_test_console.h"

#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

static const char *TAG = "bench_test_console";

struct bench_test_console_dev {
    uart_port_t uart_port;
    QueueHandle_t uart_event_queue; // owned by ESP-IDF's UART driver, same pattern as
                                     // crsf_radio_dev's equivalent field (crsf_radio.c)
    char line_buf[BENCH_TEST_COMMAND_MAX_LINE_LEN];
    size_t line_len;
};

esp_err_t bench_test_console_init(const bench_test_console_config_t *config,
                                   bench_test_console_handle_t *out_handle)
{
    if (config == NULL || out_handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    struct bench_test_console_dev *dev = calloc(1, sizeof(*dev));
    if (dev == NULL) {
        return ESP_ERR_NO_MEM;
    }
    dev->uart_port = config->uart_port;

    uart_config_t uart_cfg = {
        .baud_rate = config->baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t err = uart_param_config(config->uart_port, &uart_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed: %s", esp_err_to_name(err));
        free(dev);
        return err;
    }
    err = uart_set_pin(config->uart_port, config->tx_gpio, config->rx_gpio, UART_PIN_NO_CHANGE,
                        UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin failed: %s", esp_err_to_name(err));
        free(dev);
        return err;
    }

    size_t queue_len = (config->event_queue_length > 0) ? (size_t)config->event_queue_length : 8;
    // RX ring buffer sized well above one console line; TX buffer 0 (uart_write_bytes() blocks
    // until accepted by the driver, which is fine for this low-rate, human-facing output - see
    // bench_test_console_println()). Interrupt-driven, not polled - same reasoning as
    // crsf_radio.c's own uart_driver_install() call.
    err = uart_driver_install(config->uart_port, 256, 0, (int)queue_len, &dev->uart_event_queue, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(err));
        free(dev);
        return err;
    }

    *out_handle = dev;
    return ESP_OK;
}

static void reset_line_buf(struct bench_test_console_dev *dev)
{
    dev->line_len = 0;
}

// Feeds one received byte into the line buffer. Returns true when `dev->line_buf` holds a
// complete, NUL-terminated line ready to parse (on '\n' or '\r'); a bare '\r'/'\n' on an empty
// buffer is treated as no-op, not an empty line, so a CRLF pair doesn't produce two blank-line
// commands.
static bool feed_byte(struct bench_test_console_dev *dev, uint8_t byte, bool *out_overflowed)
{
    *out_overflowed = false;
    if (byte == '\n' || byte == '\r') {
        if (dev->line_len == 0) {
            return false;
        }
        dev->line_buf[dev->line_len] = '\0';
        return true;
    }
    if (dev->line_len + 1 >= sizeof(dev->line_buf)) {
        // Line too long: drop it and resync on the next line separator rather than silently
        // truncating and parsing a command from a corrupted prefix.
        reset_line_buf(dev);
        *out_overflowed = true;
        return false;
    }
    dev->line_buf[dev->line_len++] = (char)byte;
    return false;
}

esp_err_t bench_test_console_process_pending(
    bench_test_console_handle_t handle,
    void (*on_command)(const bench_test_command_t *cmd, void *ctx),
    void (*on_parse_error)(bench_test_parse_result_t result, const char *line, void *ctx),
    void *ctx)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Non-blocking drain of the UART driver's own event queue, same pattern as
    // crsf_radio_process_pending() - see that file for the full context-boundary writeup this
    // component follows.
    uart_event_t event;
    while (xQueueReceive(handle->uart_event_queue, &event, 0) == pdTRUE) {
        switch (event.type) {
        case UART_DATA: {
            uint8_t chunk[64];
            int n;
            while ((n = uart_read_bytes(handle->uart_port, chunk, sizeof(chunk), 0)) > 0) {
                for (int i = 0; i < n; i++) {
                    bool overflowed = false;
                    bool have_line = feed_byte(handle, chunk[i], &overflowed);
                    if (overflowed) {
                        ESP_LOGW(TAG, "console line too long (max %d chars); dropped",
                                 BENCH_TEST_COMMAND_MAX_LINE_LEN - 1);
                        if (on_parse_error != NULL) {
                            on_parse_error(BENCH_TEST_PARSE_LINE_TOO_LONG, "", ctx);
                        }
                        continue;
                    }
                    if (!have_line) {
                        continue;
                    }

                    bench_test_console_println(handle, handle->line_buf);

                    bench_test_command_t cmd;
                    bench_test_parse_result_t result =
                        bench_test_parse_line(handle->line_buf, &cmd);
                    if (result == BENCH_TEST_PARSE_OK) {
                        if (cmd.type != BENCH_TEST_CMD_NONE && on_command != NULL) {
                            on_command(&cmd, ctx);
                        }
                    } else if (on_parse_error != NULL) {
                        on_parse_error(result, handle->line_buf, ctx);
                    }

                    reset_line_buf(handle);
                }
            }
            break;
        }
        case UART_FIFO_OVF:
        case UART_BUFFER_FULL:
            ESP_LOGW(TAG, "console UART RX overflow (event=%d); flushing", (int)event.type);
            uart_flush_input(handle->uart_port);
            xQueueReset(handle->uart_event_queue);
            reset_line_buf(handle);
            break;
        default:
            break;
        }
    }

    return ESP_OK;
}

esp_err_t bench_test_console_println(bench_test_console_handle_t handle, const char *text)
{
    if (handle == NULL || text == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uart_write_bytes(handle->uart_port, text, strlen(text));
    uart_write_bytes(handle->uart_port, "\r\n", 2);
    return ESP_OK;
}

esp_err_t bench_test_console_deinit(bench_test_console_handle_t handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uart_driver_delete(handle->uart_port);
    free(handle);
    return ESP_OK;
}
