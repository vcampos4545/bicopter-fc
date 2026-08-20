// RadioTask - Milestone 6 scaffolding only. No radio transport exists yet (ESP-NOW is milestone
// 14, an RC-receiver alternative is milestone 15); this stub just runs at its target cadence and
// logs, decimated. When a real Radio implementation lands, per docs/architecture.md this task
// deposits the latest setpoint into a non-blocking mailbox for FlightControlTask to read and
// publishes the latest state for downlink - it must never let a slow/lost radio link back-pressure
// the control path.
//
// Target rate: 50Hz (RADIO_TASK_PERIOD_MS = 20, see task_config.h), mid of docs/architecture.md's
// documented 20-100Hz range (protocol-dependent - ESP-NOW vs. RC-receiver frame rate, still TBD).
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "radio_task.h"
#include "task_config.h"

static const char *TAG = "radio_task";

void radio_task(void *pvParameters)
{
    (void)pvParameters;

    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    uint32_t cycle = 0;
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        cycle++;

        if ((cycle % 50) == 1) { // ~once/second at 50Hz
            ESP_LOGI(TAG, "cycle=%lu (no radio transport yet - milestones 14-15)",
                     (unsigned long)cycle);
        }

        esp_task_wdt_reset();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(RADIO_TASK_PERIOD_MS));
    }
}
