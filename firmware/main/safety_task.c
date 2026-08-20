// SafetyTask - Milestone 6 scaffolding only. No real failsafe conditions exist to evaluate yet
// (radio-loss detection needs milestone 14/15's radio link; sensor-fault detection needs a real
// estimator; battery cutoffs need a battery monitor HAL - none of that exists before milestone
// 16). This stub re-affirms the disarmed state every cycle: writes safety_state_t via the mutex
// (safety_state.h) and pings FlightControlTask via task notification (safety_task.h) so both
// primitives are genuinely exercised on every boot, not just wired and never run.
//
// Target rate: 100Hz (SAFETY_TASK_PERIOD_MS = 10, see task_config.h) and highest priority of all
// six tasks (SAFETY_TASK_PRIORITY, see task_config.h) - per docs/architecture.md, a late failsafe
// is a crash risk, and this task is cheap to run (mostly comparisons/timestamp bookkeeping even
// once real conditions land), so high priority costs little contention against the other tasks.
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "safety_state.h"
#include "safety_task.h"
#include "task_config.h"

static const char *TAG = "safety_task";

void safety_task(void *pvParameters)
{
    safety_task_params_t *params = (safety_task_params_t *)pvParameters;

    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    uint32_t cycle = 0;
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        cycle++;

        // No real failsafe condition exists to evaluate yet (see file header) - this milestone
        // just keeps the vehicle in its startup-safe disarmed state and demonstrates the
        // mutex-write + notify path every cycle.
        safety_state_set(false, esp_timer_get_time());
        xTaskNotifyGive(params->flight_control_handle);

        if ((cycle % 100) == 1) { // ~once/second at 100Hz
            ESP_LOGI(TAG, "cycle=%lu armed=false (no failsafe conditions to evaluate yet - "
                          "milestone 16)", (unsigned long)cycle);
        }

        esp_task_wdt_reset();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SAFETY_TASK_PERIOD_MS));
    }
}
