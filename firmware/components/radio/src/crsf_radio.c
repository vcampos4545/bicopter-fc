#include "crsf_radio.h"

#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "radio_packet.h" // radio_is_stale() -- reused unchanged, see this file's header

static const char *TAG = "crsf_radio";

struct crsf_radio_dev {
    uart_port_t uart_port;
    QueueHandle_t uart_event_queue; // created by uart_driver_install() -- owned by ESP-IDF's UART
                                     // driver, not by this driver; see crsf_radio.h's file header
    crsf_channel_map_t channel_map;
    int64_t command_timeout_us;

    crsf_frame_sync_t sync; // touched only from crsf_radio_process_pending()'s task context

    // Everything below is read by radio_t's ops (any task) and written by
    // crsf_radio_process_pending() (RadioTask) -- mutex-protected for the same reason as
    // esp_now_radio_dev's equivalent fields.
    SemaphoreHandle_t mutex;
    bool has_command;
    radio_command_t last_command;
    uint32_t frame_counter; // local monotonically-increasing counter fills radio_command_t.sequence
                             // -- CRSF's wire format carries no sequence number, unlike this
                             // project's own ESP-NOW packet format (see crsf_frame.h)
    int64_t last_valid_rx_timestamp_us;
    crsf_loss_tracker_t loss_tracker;
};

esp_err_t crsf_radio_init(const crsf_radio_config_t *config, crsf_radio_handle_t *out_handle)
{
    if (config == NULL || out_handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    struct crsf_radio_dev *dev = calloc(1, sizeof(*dev));
    if (dev == NULL) {
        return ESP_ERR_NO_MEM;
    }
    dev->uart_port = config->uart_port;
    dev->channel_map = config->channel_map;
    dev->command_timeout_us = config->command_timeout_us;
    crsf_frame_sync_reset(&dev->sync);
    crsf_loss_tracker_reset(&dev->loss_tracker, config->nominal_frame_period_us);

    dev->mutex = xSemaphoreCreateMutex();
    if (dev->mutex == NULL) {
        free(dev);
        return ESP_ERR_NO_MEM;
    }

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
        goto fail;
    }
    err = uart_set_pin(config->uart_port, config->tx_gpio, config->rx_gpio, UART_PIN_NO_CHANGE,
                        UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin failed: %s", esp_err_to_name(err));
        goto fail;
    }

    size_t queue_len = (config->event_queue_length > 0) ? config->event_queue_length : 8;
    // RX ring buffer sized generously above CRSF_MAX_FRAME_SIZE so several frames can queue up
    // between RadioTask cycles without the driver overflowing. TX buffer is 0 (unused --
    // receive-only, see crsf_radio.h). Interrupt-driven (not polled): ESP-IDF's UART driver
    // installs its own ISR + ring buffer, the idiomatic pattern this milestone's design brief
    // asked for over busy-polling the UART.
    err = uart_driver_install(config->uart_port, 512, 0, (int)queue_len, &dev->uart_event_queue, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(err));
        goto fail;
    }

    *out_handle = dev;
    return ESP_OK;

fail:
    vSemaphoreDelete(dev->mutex);
    free(dev);
    return err;
}

esp_err_t crsf_radio_process_pending(crsf_radio_handle_t handle, size_t *out_frames_processed)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t frames = 0;
    uart_event_t event;
    // Non-blocking drain of the UART driver's own event queue, called once per RadioTask cycle --
    // mirrors esp_now_radio_process_pending()'s drain pattern, even though here the queue is
    // populated by ESP-IDF's UART driver ISR, not by a callback this project wrote. See
    // crsf_radio.h's file header for the full context-boundary writeup.
    while (xQueueReceive(handle->uart_event_queue, &event, 0) == pdTRUE) {
        switch (event.type) {
        case UART_DATA: {
            uint8_t chunk[64];
            int n;
            while ((n = uart_read_bytes(handle->uart_port, chunk, sizeof(chunk), 0)) > 0) {
                for (int i = 0; i < n; i++) {
                    size_t frame_len;
                    if (!crsf_frame_sync_push_byte(&handle->sync, chunk[i], &frame_len)) {
                        continue;
                    }

                    crsf_rc_channels_t channels;
                    if (!crsf_parse_rc_channels_frame(handle->sync.buf, frame_len, &channels)) {
                        ESP_LOGD(TAG, "rejected malformed/corrupted CRSF frame (len=%u)",
                                 (unsigned)frame_len);
                        continue;
                    }
                    frames++;

                    // crsf_channels_to_command() deliberately returns a local, ESP-IDF-free
                    // crsf_command_t (see crsf_frame.h) rather than radio_command_t itself; the
                    // field-by-field copy below is the same conversion esp_now_radio.c does from
                    // radio_command_packet_t.
                    crsf_command_t decoded;
                    crsf_channels_to_command(&channels, &handle->channel_map, &decoded);
                    int64_t now_us = esp_timer_get_time();

                    xSemaphoreTake(handle->mutex, portMAX_DELAY);
                    handle->has_command = true;
                    handle->last_command = (radio_command_t){
                        .throttle = decoded.throttle,
                        .roll = decoded.roll,
                        .pitch = decoded.pitch,
                        .yaw = decoded.yaw,
                        .arm = decoded.arm,
                        .flight_mode = decoded.flight_mode,
                        .sequence = handle->frame_counter++,
                        .timestamp_us = now_us, // receiver-local; see struct field comment
                    };
                    handle->last_valid_rx_timestamp_us = now_us;
                    crsf_loss_tracker_record(&handle->loss_tracker, now_us);
                    xSemaphoreGive(handle->mutex);
                }
            }
            break;
        }
        case UART_FIFO_OVF:
        case UART_BUFFER_FULL:
            // The driver's own ring buffer overran faster than we drained it. Flush and resync
            // from scratch rather than trying to recover a corrupted byte stream.
            ESP_LOGW(TAG, "UART RX overflow (event=%d); flushing and resyncing", (int)event.type);
            uart_flush_input(handle->uart_port);
            xQueueReset(handle->uart_event_queue);
            crsf_frame_sync_reset(&handle->sync);
            break;
        default:
            // UART_BREAK/UART_FRAME_ERR/UART_PARITY_ERR/UART_PATTERN_DET: a corrupted byte here
            // is already caught by CRC rejection once framing resumes; nothing else to do.
            break;
        }
    }

    if (out_frames_processed != NULL) {
        *out_frames_processed = frames;
    }
    return ESP_OK;
}

// --- radio_t ops: read/write the mutex-protected state above; never block (per radio.h's
// contract).

static bool crsf_radio_op_has_command(void *ctx)
{
    struct crsf_radio_dev *dev = (struct crsf_radio_dev *)ctx;
    int64_t now_us = esp_timer_get_time();

    xSemaphoreTake(dev->mutex, portMAX_DELAY);
    bool has = dev->has_command &&
               !radio_is_stale(dev->last_valid_rx_timestamp_us, now_us, dev->command_timeout_us);
    xSemaphoreGive(dev->mutex);
    return has;
}

static esp_err_t crsf_radio_op_get_command(void *ctx, radio_command_t *out_command)
{
    struct crsf_radio_dev *dev = (struct crsf_radio_dev *)ctx;
    if (out_command == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(dev->mutex, portMAX_DELAY);
    bool has = dev->has_command;
    *out_command = dev->last_command;
    xSemaphoreGive(dev->mutex);

    return has ? ESP_OK : ESP_ERR_NOT_FOUND;
}

static esp_err_t crsf_radio_op_get_health(void *ctx, radio_health_t *out_health)
{
    struct crsf_radio_dev *dev = (struct crsf_radio_dev *)ctx;
    if (out_health == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    int64_t now_us = esp_timer_get_time();

    xSemaphoreTake(dev->mutex, portMAX_DELAY);
    out_health->packet_loss_percent = crsf_loss_tracker_loss_percent(&dev->loss_tracker);
    out_health->last_valid_rx_timestamp_us = dev->last_valid_rx_timestamp_us;
    out_health->link_alive =
        dev->has_command && !radio_is_stale(dev->last_valid_rx_timestamp_us, now_us,
                                             dev->command_timeout_us);
    // CRSF telemetry (which would carry receiver-side link-quality/RSSI back over the return
    // channel) is out of scope this milestone -- see crsf_frame.h's file header -- so this
    // implementation has no RSSI source, unlike ESP-NOW's (whose Wi-Fi driver exposes RSSI on
    // every received frame for free).
    out_health->rssi_available = false;
    out_health->rssi_dbm = 0;
    xSemaphoreGive(dev->mutex);

    return ESP_OK;
}

esp_err_t crsf_radio_deinit(crsf_radio_handle_t handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uart_driver_delete(handle->uart_port);
    vSemaphoreDelete(handle->mutex);
    free(handle);
    return ESP_OK;
}

static esp_err_t crsf_radio_op_deinit(void *ctx)
{
    return crsf_radio_deinit((crsf_radio_handle_t)ctx);
}

static const radio_ops_t s_crsf_radio_ops = {
    .has_command = crsf_radio_op_has_command,
    .get_command = crsf_radio_op_get_command,
    .get_health = crsf_radio_op_get_health,
    .deinit = crsf_radio_op_deinit,
};

radio_t crsf_radio_as_radio(crsf_radio_handle_t handle)
{
    return (radio_t){ .ops = &s_crsf_radio_ops, .ctx = handle };
}
