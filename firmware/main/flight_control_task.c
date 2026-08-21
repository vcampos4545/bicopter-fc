// FlightControlTask - Milestone 6 scaffolding only. No attitude/rate control math or actuator
// writes exist yet (milestones 10-12 built that stack in flight_core/simulator, but wiring it into
// firmware's FlightControlTask is explicitly out of scope through Milestone 16 - see AGENTS.md);
// this task's stub body reads the mutex-protected safety_state_t (now a real flight_mode_t as of
// Milestone 16, not just a placeholder armed bool - see safety_state.h) and drains the lightweight
// "safety state changed" notification SafetyTask sends after each of its own cycles, logging both,
// decimated. Once real control/allocation wiring lands, this is also the natural place to gate
// actuator output on `safety.armed` (already false whenever SafetyTask reports FAILSAFE/DISARMED/
// BOOT/ERROR).
//
// Target rate: 250Hz (FLIGHT_CONTROL_TASK_PERIOD_MS = 4, see task_config.h), the low end of
// docs/architecture.md's documented 250-500Hz range - trailing SensorTask/EstimatorTask
// deliberately, since bicopter actuator response (ESC spin-up, servo slew) tolerates more jitter
// than the state estimate itself does.
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "flight_control_task.h"
#include "flight_mode.h"
#include "safety_state.h"
#include "task_config.h"

static const char *TAG = "flight_control_task";

void flight_control_task(void *pvParameters)
{
    (void)pvParameters;

    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    uint32_t cycle = 0;
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        cycle++;

        // Non-blocking: this task's own period already bounds its wake latency, so a pending
        // notification just tells it whether SafetyTask refreshed state since the last cycle -
        // it never needs to wait for one.
        uint32_t pending_safety_pings = ulTaskNotifyTake(pdTRUE, 0);

        safety_state_t safety;
        safety_state_get(&safety);

        if ((cycle % 250) == 1) { // ~once/second at 250Hz
            ESP_LOGI(TAG,
                     "cycle=%lu mode=%s armed=%d safety_pings=%lu (no control math yet - "
                     "milestones 10-12)",
                     (unsigned long)cycle, flight_mode_name(safety.mode), safety.armed,
                     (unsigned long)pending_safety_pings);
        }

        esp_task_wdt_reset();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(FLIGHT_CONTROL_TASK_PERIOD_MS));
    }
}
