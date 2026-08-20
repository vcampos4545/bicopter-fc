// TelemetryTask - Milestone 6 scaffolding only. No downlink transport exists yet (that rides on
// RadioTask's future radio link); this stub reads the mutex-protected safety_state_t (armed/
// disarmed - see safety_state.h) each time it wakes and logs it, decimated.
//
// Unlike the other five tasks, TelemetryTask does not pace itself with vTaskDelayUntil() - it
// blocks on ulTaskNotifyTake() and is woken by an esp_timer periodic callback (see main.c). This
// is the milestone's one esp_timer example: telemetry cadence is a pure wall-clock pacing
// requirement, independent of any other task's execution, which is exactly the case an esp_timer
// (its own dedicated high-priority "esp_timer" task, decoupled from every other task's scheduling)
// suits better than sharing this task's own delay loop. The other five tasks are periodic in their
// own right (SensorTask by hardware cadence, the rest by control/safety timing), so vTaskDelayUntil
// is the right (simpler) tool there instead - see task_config.h.
//
// Target rate: 20Hz (TELEMETRY_TASK_PERIOD_MS = 50, see task_config.h), mid of
// docs/architecture.md's documented 10-50Hz range, and lowest priority of all six tasks
// (TELEMETRY_TASK_PRIORITY) since a late telemetry/log line is never a flight-safety concern.
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "safety_state.h"
#include "telemetry_task.h"

static const char *TAG = "telemetry_task";

void telemetry_task(void *pvParameters)
{
    (void)pvParameters;

    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    uint32_t cycle = 0;

    while (1) {
        // Blocks until the esp_timer callback in main.c wakes this task; also bounds the wait so
        // this task still reliably feeds the watchdog if the timer were ever stopped/misconfigured.
        uint32_t notified = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));

        if (notified > 0) {
            cycle++;

            safety_state_t safety;
            safety_state_get(&safety);

            if ((cycle % 20) == 1) { // ~once/second at 20Hz
                ESP_LOGI(TAG, "cycle=%lu armed=%d (no downlink transport yet - milestone 14+)",
                         (unsigned long)cycle, safety.armed);
            }
        }

        esp_task_wdt_reset();
    }
}
