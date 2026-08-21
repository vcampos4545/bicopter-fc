// RadioTask - Milestone 14 wires in the real ESP-NOW radio (firmware/components/radio/), gated
// behind CONFIG_BICOPTER_RADIO_ENABLED (default off: no ground-station peer has been paired in
// this environment -- same "no board/hardware yet" pattern as SensorTask's
// CONFIG_BICOPTER_SENSORS_ENABLED, see sensor_task.c). With it disabled, this task still runs its
// full loop/watchdog plumbing and just logs, exactly like Milestone 6's stub did.
//
// esp_now_radio_process_pending() is this milestone's "dedicated task" per docs/architecture.md's
// ISR/task-context split: ESP-NOW's receive callback (esp_now_recv_cb(), in
// firmware/components/radio/src/esp_now_radio.c) only ever posts a minimally-validated raw packet
// to a queue from the Wi-Fi driver's own task context -- all real parsing, sequence-staleness/
// reordering rejection, and packet-loss tracking happens here, in RadioTask, once per cycle. See
// docs/radio.md for the full ISR/task-context writeup and packet format.
//
// Target rate: 50Hz (RADIO_TASK_PERIOD_MS = 20, see task_config.h), mid of docs/architecture.md's
// documented 20-100Hz range (protocol-dependent - ESP-NOW vs. RC-receiver frame rate, still TBD).
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sdkconfig.h"

#include "radio_task.h"
#include "task_config.h"

#if CONFIG_BICOPTER_RADIO_ENABLED
#include <stdio.h>

#include "esp_now_radio.h"
#endif

static const char *TAG = "radio_task";

#if CONFIG_BICOPTER_RADIO_ENABLED
// Real hardware init path. Reviewed against ESP-IDF's ESP-NOW API docs, not exercised: no
// physical ESP32 pair was available in this environment, and CONFIG_BICOPTER_RADIO_ENABLED
// defaults to "n" for exactly that reason (see docs/radio.md's pairing procedure and
// verified-vs-deferred section).
static esp_err_t radio_hw_init(esp_now_radio_handle_t *out_handle)
{
    unsigned mac[6];
    int n = sscanf(CONFIG_BICOPTER_RADIO_PEER_MAC, "%2x:%2x:%2x:%2x:%2x:%2x", &mac[0], &mac[1],
                   &mac[2], &mac[3], &mac[4], &mac[5]);
    if (n != 6) {
        ESP_LOGE(TAG, "CONFIG_BICOPTER_RADIO_PEER_MAC='%s' is not a valid AA:BB:CC:DD:EE:FF MAC",
                 CONFIG_BICOPTER_RADIO_PEER_MAC);
        return ESP_ERR_INVALID_ARG;
    }

    esp_now_radio_config_t cfg = {
        .wifi_channel = CONFIG_BICOPTER_RADIO_WIFI_CHANNEL,
        .command_timeout_us = (int64_t)CONFIG_BICOPTER_RADIO_COMMAND_TIMEOUT_MS * 1000,
        .rx_queue_length = 8,
    };
    for (int i = 0; i < 6; i++) {
        cfg.peer_mac[i] = (uint8_t)mac[i];
    }

    return esp_now_radio_init(&cfg, out_handle);
}
#endif // CONFIG_BICOPTER_RADIO_ENABLED

void radio_task(void *pvParameters)
{
    (void)pvParameters;

    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

#if CONFIG_BICOPTER_RADIO_ENABLED
    esp_now_radio_handle_t esp_now_handle = NULL;
    radio_t radio = { 0 };
    bool radio_ready = false;

    if (radio_hw_init(&esp_now_handle) == ESP_OK) {
        radio = esp_now_radio_as_radio(esp_now_handle);
        radio_ready = true;
        ESP_LOGI(TAG, "ESP-NOW radio initialized (channel=%d, timeout=%dms)",
                 CONFIG_BICOPTER_RADIO_WIFI_CHANNEL, CONFIG_BICOPTER_RADIO_COMMAND_TIMEOUT_MS);
    } else {
        ESP_LOGW(TAG, "ESP-NOW radio init failed; running task loop without a radio transport");
    }
#else
    ESP_LOGI(TAG,
             "CONFIG_BICOPTER_RADIO_ENABLED=n (no peer paired yet, see docs/radio.md); "
             "running the task/watchdog loop without real radio I/O");
#endif

    uint32_t cycle = 0;
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        cycle++;

#if CONFIG_BICOPTER_RADIO_ENABLED
        if (radio_ready) {
            size_t processed = 0;
            esp_now_radio_process_pending(esp_now_handle, &processed);
        }
#endif

        if ((cycle % 50) == 1) { // ~once/second at 50Hz
#if CONFIG_BICOPTER_RADIO_ENABLED
            if (radio_ready) {
                radio_health_t health;
                radio_get_health(&radio, &health);
                ESP_LOGI(TAG,
                         "cycle=%lu link_alive=%d loss=%.1f%% rssi=%d(avail=%d)",
                         (unsigned long)cycle, health.link_alive, (double)health.packet_loss_percent,
                         health.rssi_dbm, health.rssi_available);
            } else {
                ESP_LOGI(TAG, "cycle=%lu (radio init failed - see docs/radio.md)",
                         (unsigned long)cycle);
            }
#else
            ESP_LOGI(TAG, "cycle=%lu (no radio transport - CONFIG_BICOPTER_RADIO_ENABLED=n)",
                     (unsigned long)cycle);
#endif
        }

        esp_task_wdt_reset();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(RADIO_TASK_PERIOD_MS));
    }
}
