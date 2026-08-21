#include "bench_test_task.h"

#include <inttypes.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "sdkconfig.h"

#include "bench_test_command.h"
#include "bench_test_console.h"

#include "actuators_state.h"
#include "radio_state.h"
#include "sensor_task.h"
#include "task_config.h"

static const char *TAG = "bench_test_task";

#define BICOPTER_DEG_TO_RAD(deg) ((deg) * 0.017453292519943295f)
#define BICOPTER_RAD_TO_DEG(rad) ((rad) * 57.29577951308232f)

typedef struct {
    bench_test_console_handle_t console;
    QueueHandle_t sensor_sample_queue;
    bool sensor_streaming;
    bool radio_streaming;
} bench_test_state_t;

static void print_line(bench_test_console_handle_t console, const char *fmt, ...)
{
    char buf[160];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    bench_test_console_println(console, buf);
}

static void handle_servo(bench_test_state_t *state, const bench_test_command_t *cmd)
{
#if CONFIG_BICOPTER_OPERATING_MODE_FLIGHT
    print_line(state->console,
               "servo: refused - not available in a FLIGHT build (bypasses the control loop; "
               "see docs/bench_test.md)");
    ESP_LOGW(TAG, "servo command refused: FLIGHT build");
    return;
#else
    actuators_t actuators;
    if (!actuators_state_get(&actuators)) {
        print_line(state->console,
                   "servo: actuators not initialized (CONFIG_BICOPTER_ACTUATORS_ENABLED=n or init "
                   "failed - see docs/hardware.md)");
        return;
    }
    if (cmd->unit < 0 || cmd->unit >= ACTUATORS_NUM_UNITS) {
        print_line(state->console, "servo: invalid unit %d", cmd->unit);
        return;
    }
    float angle_rad = BICOPTER_DEG_TO_RAD(cmd->value);
    esp_err_t err = servo_output_write(actuators.units[cmd->unit].servo, angle_rad);
    print_line(state->console, "servo %d -> %.2f deg: %s", cmd->unit, (double)cmd->value,
               esp_err_to_name(err));
    ESP_LOGI(TAG, "servo %d commanded to %.2f deg (%s)", cmd->unit, (double)cmd->value,
             esp_err_to_name(err));
#endif
}

static void handle_esc_test(bench_test_state_t *state, const bench_test_command_t *cmd)
{
    // Loudly logged on every invocation, regardless of whether it's actually honored below - the
    // design brief's explicit requirement (see docs/bench_test.md). cmd->confirmed is always true
    // here by bench_test_parse_line()'s contract (an unconfirmed line never parses as
    // BENCH_TEST_CMD_ESC_TEST), so this message alone documents that the operator did type the
    // required CONFIRM token.
    ESP_LOGW(TAG,
             "*** BENCH TEST: esc_test invoked (unit=%d throttle=%.3f confirmed=%d) *** ENSURE "
             "PROPELLERS ARE REMOVED - see docs/bench_test.md",
             cmd->unit, (double)cmd->value, cmd->confirmed);

#if !CONFIG_BICOPTER_OPERATING_MODE_HARDWARE_TEST
    print_line(state->console,
               "esc_test: refused - only available in a HARDWARE_TEST build "
               "(CONFIG_BICOPTER_OPERATING_MODE_HARDWARE_TEST); see docs/bench_test.md");
    ESP_LOGW(TAG, "esc_test refused: build is not HARDWARE_TEST mode");
    return;
#else
    actuators_t actuators;
    if (!actuators_state_get(&actuators)) {
        print_line(state->console,
                   "esc_test: actuators not initialized (CONFIG_BICOPTER_ACTUATORS_ENABLED=n or "
                   "init failed - see docs/hardware.md)");
        return;
    }
    if (cmd->unit < 0 || cmd->unit >= ACTUATORS_NUM_UNITS) {
        print_line(state->console, "esc_test: invalid unit %d", cmd->unit);
        return;
    }

    float max_throttle = (float)CONFIG_BICOPTER_BENCH_TEST_ESC_MAX_THROTTLE_PERCENT / 100.0f;
    float clamped = cmd->value;
    if (clamped > max_throttle) {
        clamped = max_throttle;
    }
    if (clamped < 0.0f) {
        clamped = 0.0f;
    }

    print_line(state->console,
               "esc_test: motor %d -> throttle %.3f for %dms, then auto-idle "
               "(requested %.3f, clamped to CONFIG_BICOPTER_BENCH_TEST_ESC_MAX_THROTTLE_PERCENT=%d%%)",
               cmd->unit, (double)clamped, CONFIG_BICOPTER_BENCH_TEST_ESC_TEST_DURATION_MS,
               (double)cmd->value, CONFIG_BICOPTER_BENCH_TEST_ESC_MAX_THROTTLE_PERCENT);
    ESP_LOGW(TAG, "*** BENCH TEST: commanding motor %d to throttle %.3f ***", cmd->unit,
             (double)clamped);

    esp_err_t err = motor_output_write(&actuators.units[cmd->unit].motor, clamped);
    if (err != ESP_OK) {
        print_line(state->console, "esc_test: motor_output_write failed: %s",
                   esp_err_to_name(err));
        ESP_LOGE(TAG, "esc_test motor_output_write failed: %s", esp_err_to_name(err));
        return;
    }

    // Blocks this task for the configured duration, then unconditionally returns to idle - a
    // confirmed esc_test can never leave a motor spinning because an operator forgot a second
    // "stop" command. This task is not watchdog-registered and has no fixed period (see
    // bench_test_task.h), so blocking here does not risk a watchdog panic or stall any other
    // task.
    vTaskDelay(pdMS_TO_TICKS(CONFIG_BICOPTER_BENCH_TEST_ESC_TEST_DURATION_MS));

    esp_err_t idle_err = motor_output_set_idle(&actuators.units[cmd->unit].motor);
    print_line(state->console, "esc_test: motor %d returned to idle: %s", cmd->unit,
               esp_err_to_name(idle_err));
    ESP_LOGW(TAG, "BENCH TEST: motor %d returned to idle (%s)", cmd->unit,
             esp_err_to_name(idle_err));
#endif
}

static void on_command(const bench_test_command_t *cmd, void *ctx)
{
    bench_test_state_t *state = (bench_test_state_t *)ctx;

    switch (cmd->type) {
    case BENCH_TEST_CMD_HELP:
        bench_test_console_println(state->console, bench_test_help_text());
        break;
    case BENCH_TEST_CMD_SENSOR_START:
        state->sensor_streaming = true;
        print_line(state->console, "sensor: streaming started");
        break;
    case BENCH_TEST_CMD_SENSOR_STOP:
        state->sensor_streaming = false;
        print_line(state->console, "sensor: streaming stopped");
        break;
    case BENCH_TEST_CMD_RADIO_START:
        state->radio_streaming = true;
        print_line(state->console, "radio: streaming started");
        break;
    case BENCH_TEST_CMD_RADIO_STOP:
        state->radio_streaming = false;
        print_line(state->console, "radio: streaming stopped");
        break;
    case BENCH_TEST_CMD_SERVO:
        handle_servo(state, cmd);
        break;
    case BENCH_TEST_CMD_ESC_TEST:
        handle_esc_test(state, cmd);
        break;
    case BENCH_TEST_CMD_NONE:
    default:
        break;
    }
}

static void on_parse_error(bench_test_parse_result_t result, const char *line, void *ctx)
{
    bench_test_state_t *state = (bench_test_state_t *)ctx;
    (void)line;
    print_line(state->console, "error: %s - type 'help' for the command list",
               bench_test_parse_result_name(result));
}

static void print_sensor_sample(bench_test_state_t *state)
{
    if (state->sensor_sample_queue == NULL) {
        return;
    }
    sensor_sample_t sample;
    memset(&sample, 0, sizeof(sample));
    if (xQueuePeek(state->sensor_sample_queue, &sample, 0) != pdTRUE) {
        return;
    }
    print_line(state->console,
               "sensor: imu.valid=%d accel=(%.3f,%.3f,%.3f) gyro=(%.3f,%.3f,%.3f) | "
               "baro.valid=%d pressure_pa=%.1f temp_c=%.2f alt_m=%.2f",
               sample.imu.valid, (double)sample.imu.accel.x, (double)sample.imu.accel.y,
               (double)sample.imu.accel.z, (double)sample.imu.gyro.x, (double)sample.imu.gyro.y,
               (double)sample.imu.gyro.z, sample.baro.valid, (double)sample.baro.pressure_pa,
               (double)sample.baro.temperature_c, (double)sample.baro.altitude_m);
}

static void print_radio_command(bench_test_state_t *state)
{
    radio_state_t radio;
    radio_state_get(&radio);
    if (!radio.available) {
        print_line(state->console, "radio: no transport enabled this build");
        return;
    }
    if (!radio.has_command) {
        print_line(state->console, "radio: no command received yet (link_alive=%d)",
                   radio.health.link_alive);
        return;
    }
    print_line(state->console,
               "radio: throttle=%.3f roll=%.3f pitch=%.3f yaw=%.3f arm=%d flight_mode=%d "
               "link_alive=%d loss=%.1f%%",
               (double)radio.command.throttle, (double)radio.command.roll,
               (double)radio.command.pitch, (double)radio.command.yaw, radio.command.arm,
               radio.command.flight_mode, radio.health.link_alive,
               (double)radio.health.packet_loss_percent);
}

void bench_test_task(void *pvParameters)
{
    bench_test_task_params_t *params = (bench_test_task_params_t *)pvParameters;

    bench_test_state_t state = {0};
    state.sensor_sample_queue = params->sensor_sample_queue;

    bench_test_console_config_t console_cfg = {
        .uart_port = (uart_port_t)CONFIG_BICOPTER_BENCH_TEST_CONSOLE_UART_PORT,
        .rx_gpio = CONFIG_BICOPTER_BENCH_TEST_CONSOLE_RX_GPIO,
        .tx_gpio = CONFIG_BICOPTER_BENCH_TEST_CONSOLE_TX_GPIO,
        .baud_rate = CONFIG_BICOPTER_BENCH_TEST_CONSOLE_BAUD_RATE,
        .event_queue_length = 8,
    };
    esp_err_t err = bench_test_console_init(&console_cfg, &state.console);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "bench_test_console_init failed: %s; BenchTestTask exiting",
                 esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "bench-test console ready on UART%d (rx=%d tx=%d baud=%d) - type 'help'",
             CONFIG_BICOPTER_BENCH_TEST_CONSOLE_UART_PORT, CONFIG_BICOPTER_BENCH_TEST_CONSOLE_RX_GPIO,
             CONFIG_BICOPTER_BENCH_TEST_CONSOLE_TX_GPIO, CONFIG_BICOPTER_BENCH_TEST_CONSOLE_BAUD_RATE);
    bench_test_console_println(state.console, "bicopter-fc bench-test console ready. Type 'help'.");

    while (1) {
        bench_test_console_process_pending(state.console, on_command, on_parse_error, &state);

        if (state.sensor_streaming) {
            print_sensor_sample(&state);
        }
        if (state.radio_streaming) {
            print_radio_command(&state);
        }

        // Not watchdog-registered (see bench_test_task.h) - this delay just sets the console's
        // poll/streaming cadence, not a deadline anything else depends on.
        vTaskDelay(pdMS_TO_TICKS(BENCH_TEST_TASK_PERIOD_MS));
    }
}
