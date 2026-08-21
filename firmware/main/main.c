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

    // -> actuators are intentionally NOT initialized here. actuators_init_safe()
    // (firmware/components/actuators/include/actuators_init.h) exists and its own docs name this
    // milestone as where it gets wired in, but doing so needs a real per-board GPIO/PWM config
    // (see docs/hardware.md's open ESC/servo pin items) that does not exist without a chosen
    // board, and this milestone's scope explicitly excludes actuator-driving logic from the task
    // bodies. The natural call site (before FlightControlTask starts, so every motor/servo is
    // idle/neutral before any control-path task runs) is documented here for whichever milestone
    // finalizes the board config to wire in without re-deriving where it goes.
    ESP_LOGI(TAG, "actuators_init_safe() not called (no board/PWM pin config chosen yet - see "
                  "docs/hardware.md); actuators remain uninitialized this boot");

    // -> tasks. SensorTask and EstimatorTask share the sample queue; SafetyTask needs
    // FlightControlTask's handle to notify it, so FlightControlTask is created first.
    static sensor_task_params_t sensor_params;
    sensor_params.sample_queue = s_sensor_sample_queue;

    static estimator_task_params_t estimator_params;
    estimator_params.sample_queue = s_sensor_sample_queue;

    static safety_task_params_t safety_params;
    safety_params.sensor_sample_queue = s_sensor_sample_queue;

    TaskHandle_t flight_control_handle = NULL;
    TaskHandle_t sensor_handle = NULL;
    TaskHandle_t estimator_handle = NULL;
    TaskHandle_t radio_handle = NULL;
    TaskHandle_t safety_handle = NULL;

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

    if (flight_control_handle == NULL || sensor_handle == NULL || estimator_handle == NULL ||
        radio_handle == NULL || s_telemetry_task_handle == NULL || safety_handle == NULL) {
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
    ESP_LOGI(TAG, "bicopter-fc ready: 6 tasks running (SafetyTask, SensorTask, EstimatorTask, "
                  "FlightControlTask, RadioTask, TelemetryTask)");

    // app_main() returning deletes the main task (standard ESP-IDF behavior) - everything from
    // here on runs in the six tasks created above.
}
