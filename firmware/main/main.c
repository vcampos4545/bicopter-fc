// main/: FreeRTOS entry point and composition root. Milestone 6 wires up the task architecture
// docs/architecture.md sketched in Milestone 1: creates the shared queue/mutex/timer, starts all
// six tasks at their documented priorities/periods, and enables the task watchdog. Task bodies
// remain stub/no-op logic (see each task_*.c file and AGENTS.md "What's real vs. stub right now")
// - this file only owns the wiring, not any flight-control behavior.
#include "esp_idf_version.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "sdkconfig.h"

#include "actuators_init.h"
#include "actuators_state.h"
#include "bench_test_task.h"
#include "estimator_task.h"
#include "flight_control_task.h"
#include "radio_state.h"
#include "radio_task.h"
#include "safety_state.h"
#include "safety_task.h"
#include "sensor_task.h"
#include "task_config.h"
#include "telemetry_task.h"

static const char *TAG = "bicopter-fc";

static QueueHandle_t s_sensor_sample_queue;
static TaskHandle_t s_telemetry_task_handle;

// esp_timer callback driving TelemetryTask's wake (see telemetry_task.c for why this one task
// uses a timer instead of vTaskDelayUntil). On plain esp32, esp_timer callbacks run in the
// dedicated "esp_timer" service task (ESP_TASK_TIMER_PRIO = configMAX_PRIORITIES-3, near the top
// of this project's priority range) rather than interrupt context - ESP32 only supports esp_timer's
// TASK dispatch method (ISR dispatch is an ESP32-C2-only feature, see esp_timer's Kconfig). That
// means this callback is real task context, not an ISR: a plain xTaskNotifyGive() is correct here,
// not the FromISR variant, and there is no portYIELD_FROM_ISR to issue.
static void telemetry_timer_cb(void *arg)
{
    (void)arg;
    xTaskNotifyGive(s_telemetry_task_handle);
}

void app_main(void)
{
    // BOOT
    ESP_LOGI(TAG, "bicopter-fc firmware starting (ESP-IDF %s)", esp_get_idf_version());

    // -> shared primitives, created before any task that uses them.
    // Queue: length-1 "latest sample" mailbox for SensorTask -> EstimatorTask handoff (see
    // sensor_task.h for why overwrite semantics, not a blocking multi-slot queue).
    s_sensor_sample_queue = xQueueCreate(1, sizeof(sensor_sample_t));
    if (s_sensor_sample_queue == NULL) {
        ESP_LOGE(TAG, "failed to create sensor sample queue");
        abort();
    }

    // Mutexes: the genuinely cross-task shared state (safety_state.h, and - as of Milestone 16 -
    // radio_state.h, publishing RadioTask's latest command/health for SafetyTask's real arming/
    // failsafe evaluation - see radio_state.h).
    ESP_ERROR_CHECK(safety_state_init());
    ESP_ERROR_CHECK(radio_state_init());

    // -> actuators. Milestone 18 (see docs/bench_test.md) finally calls actuators_init_safe() at
    // the call site AGENTS.md's Milestone 5/6 entries already documented - before any control-path
    // task runs, so every motor/servo is idle/neutral first - but only when
    // CONFIG_BICOPTER_ACTUATORS_ENABLED is set (default off, same "no board chosen yet" gating
    // every other optional-hardware component in this project uses). While disabled, actuators
    // remain uninitialized this boot exactly as every milestone through 17 has left them; the
    // bench-test console's servo/esc_test commands (bench_test_task.c) report "not initialized"
    // rather than attempting to drive a GPIO nothing is connected to.
#if CONFIG_BICOPTER_ACTUATORS_ENABLED
    actuators_config_t actuators_cfg = {
        .units = {
            [0] = {
                .motor = {
                    .gpio = (gpio_num_t)CONFIG_BICOPTER_MOTOR1_GPIO,
                    .ledc_timer = LEDC_TIMER_0,
                    .ledc_channel = LEDC_CHANNEL_0,
                    .ledc_speed_mode = LEDC_LOW_SPEED_MODE,
                    .freq_hz = CONFIG_BICOPTER_ACTUATOR_PWM_FREQ_HZ,
                    .duty_resolution_bits = CONFIG_BICOPTER_ACTUATOR_PWM_DUTY_RESOLUTION_BITS,
                    .convert = {
                        .min_throttle = 0.0f,
                        .max_throttle = 1.0f,
                        .min_pulse_us = CONFIG_BICOPTER_ESC_MIN_PULSE_US,
                        .max_pulse_us = CONFIG_BICOPTER_ESC_MAX_PULSE_US,
                        .idle_pulse_us = CONFIG_BICOPTER_ESC_IDLE_PULSE_US,
                        .invert_direction = false,
                    },
                },
                .servo = {
                    .gpio = (gpio_num_t)CONFIG_BICOPTER_SERVO1_GPIO,
                    .ledc_timer = LEDC_TIMER_1,
                    .ledc_channel = LEDC_CHANNEL_2,
                    .ledc_speed_mode = LEDC_LOW_SPEED_MODE,
                    .freq_hz = CONFIG_BICOPTER_ACTUATOR_PWM_FREQ_HZ,
                    .duty_resolution_bits = CONFIG_BICOPTER_ACTUATOR_PWM_DUTY_RESOLUTION_BITS,
                    .convert = {
                        .min_angle_rad = (float)CONFIG_BICOPTER_SERVO_MIN_ANGLE_MILLIRAD / 1000.0f,
                        .max_angle_rad = (float)CONFIG_BICOPTER_SERVO_MAX_ANGLE_MILLIRAD / 1000.0f,
                        .neutral_angle_rad =
                            (float)CONFIG_BICOPTER_SERVO_NEUTRAL_ANGLE_MILLIRAD / 1000.0f,
                        .min_pulse_us = CONFIG_BICOPTER_SERVO_MIN_PULSE_US,
                        .max_pulse_us = CONFIG_BICOPTER_SERVO_MAX_PULSE_US,
                        .invert_direction = false,
                    },
                },
            },
            [1] = {
                .motor = {
                    .gpio = (gpio_num_t)CONFIG_BICOPTER_MOTOR2_GPIO,
                    .ledc_timer = LEDC_TIMER_0,
                    .ledc_channel = LEDC_CHANNEL_1,
                    .ledc_speed_mode = LEDC_LOW_SPEED_MODE,
                    .freq_hz = CONFIG_BICOPTER_ACTUATOR_PWM_FREQ_HZ,
                    .duty_resolution_bits = CONFIG_BICOPTER_ACTUATOR_PWM_DUTY_RESOLUTION_BITS,
                    .convert = {
                        .min_throttle = 0.0f,
                        .max_throttle = 1.0f,
                        .min_pulse_us = CONFIG_BICOPTER_ESC_MIN_PULSE_US,
                        .max_pulse_us = CONFIG_BICOPTER_ESC_MAX_PULSE_US,
                        .idle_pulse_us = CONFIG_BICOPTER_ESC_IDLE_PULSE_US,
                        .invert_direction = false,
                    },
                },
                .servo = {
                    .gpio = (gpio_num_t)CONFIG_BICOPTER_SERVO2_GPIO,
                    .ledc_timer = LEDC_TIMER_1,
                    .ledc_channel = LEDC_CHANNEL_3,
                    .ledc_speed_mode = LEDC_LOW_SPEED_MODE,
                    .freq_hz = CONFIG_BICOPTER_ACTUATOR_PWM_FREQ_HZ,
                    .duty_resolution_bits = CONFIG_BICOPTER_ACTUATOR_PWM_DUTY_RESOLUTION_BITS,
                    .convert = {
                        .min_angle_rad = (float)CONFIG_BICOPTER_SERVO_MIN_ANGLE_MILLIRAD / 1000.0f,
                        .max_angle_rad = (float)CONFIG_BICOPTER_SERVO_MAX_ANGLE_MILLIRAD / 1000.0f,
                        .neutral_angle_rad =
                            (float)CONFIG_BICOPTER_SERVO_NEUTRAL_ANGLE_MILLIRAD / 1000.0f,
                        .min_pulse_us = CONFIG_BICOPTER_SERVO_MIN_PULSE_US,
                        .max_pulse_us = CONFIG_BICOPTER_SERVO_MAX_PULSE_US,
                        .invert_direction = false,
                    },
                },
            },
        },
    };
    actuators_t actuators;
    esp_err_t actuators_err = actuators_init_safe(&actuators_cfg, &actuators);
    if (actuators_err == ESP_OK) {
        actuators_state_set(&actuators, true);
        ESP_LOGI(TAG, "actuators initialized (motors idle, servos neutral)");
    } else {
        actuators_state_set(NULL, false);
        ESP_LOGE(TAG, "actuators_init_safe failed: %s; actuators unavailable this boot",
                 esp_err_to_name(actuators_err));
    }
#else
    actuators_state_set(NULL, false);
    ESP_LOGI(TAG, "CONFIG_BICOPTER_ACTUATORS_ENABLED=n (no board/PWM pin config chosen yet - see "
                  "docs/hardware.md); actuators remain uninitialized this boot");
#endif

    // -> tasks. SensorTask and EstimatorTask share the sample queue; SafetyTask needs
    // FlightControlTask's handle to notify it, so FlightControlTask is created first.
    static sensor_task_params_t sensor_params;
    sensor_params.sample_queue = s_sensor_sample_queue;

    static estimator_task_params_t estimator_params;
    estimator_params.sample_queue = s_sensor_sample_queue;

    static safety_task_params_t safety_params;
    safety_params.sensor_sample_queue = s_sensor_sample_queue;

    static bench_test_task_params_t bench_test_params;
    bench_test_params.sensor_sample_queue = s_sensor_sample_queue;

    TaskHandle_t flight_control_handle = NULL;
    TaskHandle_t sensor_handle = NULL;
    TaskHandle_t estimator_handle = NULL;
    TaskHandle_t radio_handle = NULL;
    TaskHandle_t safety_handle = NULL;
    TaskHandle_t bench_test_handle = NULL;

    xTaskCreate(flight_control_task, "FlightControlTask", BICOPTER_TASK_STACK_SIZE_BYTES, NULL,
                FLIGHT_CONTROL_TASK_PRIORITY, &flight_control_handle);
    safety_params.flight_control_handle = flight_control_handle;

    xTaskCreate(sensor_task, "SensorTask", BICOPTER_TASK_STACK_SIZE_BYTES, &sensor_params,
                SENSOR_TASK_PRIORITY, &sensor_handle);
    xTaskCreate(estimator_task, "EstimatorTask", BICOPTER_TASK_STACK_SIZE_BYTES, &estimator_params,
                ESTIMATOR_TASK_PRIORITY, &estimator_handle);
    xTaskCreate(radio_task, "RadioTask", BICOPTER_TASK_STACK_SIZE_BYTES, NULL, RADIO_TASK_PRIORITY,
                &radio_handle);
    xTaskCreate(telemetry_task, "TelemetryTask", BICOPTER_TASK_STACK_SIZE_BYTES, NULL,
                TELEMETRY_TASK_PRIORITY, &s_telemetry_task_handle);
    xTaskCreate(safety_task, "SafetyTask", BICOPTER_TASK_STACK_SIZE_BYTES, &safety_params,
                SAFETY_TASK_PRIORITY, &safety_handle);
    xTaskCreate(bench_test_task, "BenchTestTask", BICOPTER_TASK_STACK_SIZE_BYTES, &bench_test_params,
                BENCH_TEST_TASK_PRIORITY, &bench_test_handle);

    if (flight_control_handle == NULL || sensor_handle == NULL || estimator_handle == NULL ||
        radio_handle == NULL || s_telemetry_task_handle == NULL || safety_handle == NULL ||
        bench_test_handle == NULL) {
        ESP_LOGE(TAG, "one or more tasks failed to create");
        abort();
    }

    // -> timer: drives TelemetryTask's wake (see telemetry_timer_cb() above and telemetry_task.c).
    const esp_timer_create_args_t telemetry_timer_args = {
        .callback = &telemetry_timer_cb,
        .name = "telemetry_wake",
    };
    esp_timer_handle_t telemetry_timer;
    ESP_ERROR_CHECK(esp_timer_create(&telemetry_timer_args, &telemetry_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(telemetry_timer, TELEMETRY_TASK_PERIOD_MS * 1000));

    ESP_LOGI(TAG, "task watchdog: enabled at boot via CONFIG_ESP_TASK_WDT_INIT (see "
                  "sdkconfig.defaults); each task subscribes itself via esp_task_wdt_add(NULL)");

    // -> log ready
#if CONFIG_BICOPTER_OPERATING_MODE_SIMULATION
    ESP_LOGI(TAG, "operating mode: SIMULATION (see docs/bench_test.md)");
#elif CONFIG_BICOPTER_OPERATING_MODE_HARDWARE_TEST
    ESP_LOGI(TAG, "operating mode: HARDWARE_TEST (see docs/bench_test.md)");
#elif CONFIG_BICOPTER_OPERATING_MODE_FLIGHT
    ESP_LOGI(TAG, "operating mode: FLIGHT (see docs/bench_test.md)");
#endif
    ESP_LOGI(TAG, "bicopter-fc ready: 7 tasks running (SafetyTask, SensorTask, EstimatorTask, "
                  "FlightControlTask, RadioTask, TelemetryTask, BenchTestTask)");

    // app_main() returning deletes the main task (standard ESP-IDF behavior) - everything from
    // here on runs in the seven tasks created above.
}
