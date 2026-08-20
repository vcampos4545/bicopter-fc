// EstimatorTask - Milestone 6 scaffolding only. Receives each sensor_sample_t SensorTask
// publishes and logs/discards it (decimated, to avoid flooding the log at task rate) - there is no
// estimation math to feed it yet (flight_core/estimation/ lands in milestones 7-8). Per
// docs/architecture.md, this task is deliberately queue-driven rather than given its own periodic
// timer: it blocks on the sensor queue and runs immediately when a new sample exists, so no
// artificial latency is added between "gyro sample exists" and "the eventual estimate reflects
// it". ESTIMATOR_TASK_QUEUE_WAIT_MS only bounds how long a single receive call blocks before
// looping back to feed the watchdog if SensorTask stalls.
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"

#include "estimator_task.h"
#include "sensor_task.h"
#include "task_config.h"

static const char *TAG = "estimator_task";

void estimator_task(void *pvParameters)
{
    estimator_task_params_t *params = (estimator_task_params_t *)pvParameters;

    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    sensor_sample_t sample;
    uint32_t received = 0;

    while (1) {
        if (xQueueReceive(params->sample_queue, &sample, pdMS_TO_TICKS(ESTIMATOR_TASK_QUEUE_WAIT_MS)) ==
            pdTRUE) {
            received++;
            if ((received % 500) == 1) { // ~once/second at 500Hz
                ESP_LOGI(TAG, "received=%lu imu.valid=%d baro.valid=%d (no estimation math yet - "
                              "milestones 7-8)", (unsigned long)received, sample.imu.valid,
                         sample.baro.valid);
            }
        }
        // else: no new sample within the wait window - nothing to do yet; loop back around so
        // this task still reliably feeds the watchdog even if SensorTask stalls.

        esp_task_wdt_reset();
    }
}
