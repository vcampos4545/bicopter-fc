// RadioTask - Milestone 14 wired in the real ESP-NOW radio (firmware/components/radio/), gated
// behind CONFIG_BICOPTER_RADIO_ENABLED. Milestone 15 adds a second, mutually exclusive backend:
// CRSF over a UART-connected RC receiver, gated behind CONFIG_BICOPTER_CRSF_RADIO_ENABLED. Both
// implement the same radio_t interface (radio.h) unchanged, so everything below the `#if
// CONFIG_BICOPTER_*_RADIO_ENABLED` branches - the health-log cadence, the watchdog/period loop -
// is shared, transport-agnostic code. Enabling both options at once is a build-time error (see
// the #error guard below): only one radio transport can back this task's single `radio_t` at a
// time until a real board decides otherwise (Milestone 17). With neither enabled, this task still
// runs its full loop/watchdog plumbing and just logs, exactly like Milestone 6's stub did.
//
// esp_now_radio_process_pending()/crsf_radio_process_pending() are each backend's "dedicated
// task" per docs/architecture.md's ISR/task-context split: ESP-NOW's receive callback and the
// UART driver's own ISR each only ever hand off minimally-processed data to a queue from foreign
// context - all real parsing, validation, and staleness/loss tracking happens here, in RadioTask,
// once per cycle. See docs/radio.md for the full write-up (both backends' context boundaries and
// packet/frame formats).
//
// Target rate: 50Hz (RADIO_TASK_PERIOD_MS = 20, see task_config.h), mid of docs/architecture.md's
// documented 20-100Hz range (protocol-dependent - ESP-NOW vs. RC-receiver frame rate, still TBD).
//
// Milestone 16 adds one line of real wiring: every cycle, the latest decoded command and link
// health are published via radio_state_set() (radio_state.h) so SafetyTask can evaluate real
// arming/failsafe preconditions against them - the first live consumer of this task's output by
// another task (see AGENTS.md's Milestone 14 note that RadioTask "does not yet deposit setpoints
// into FlightControlTask"; that larger setpoint-plumbing item is still unstarted, this is a
// narrower, safety-only publish).
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sdkconfig.h"

#include "radio_state.h"
#include "radio_task.h"
#include "task_config.h"

#if CONFIG_BICOPTER_RADIO_ENABLED && CONFIG_BICOPTER_CRSF_RADIO_ENABLED
#error "CONFIG_BICOPTER_RADIO_ENABLED (ESP-NOW) and CONFIG_BICOPTER_CRSF_RADIO_ENABLED (CRSF) " \
       "cannot both be enabled: RadioTask backs a single radio_t with one transport at a time. " \
       "See docs/radio.md."
#endif

#if CONFIG_BICOPTER_RADIO_ENABLED
#include <stdio.h>

#include "esp_now_radio.h"
#elif CONFIG_BICOPTER_CRSF_RADIO_ENABLED
#include "crsf_radio.h"
#endif

#define RADIO_TRANSPORT_ENABLED (CONFIG_BICOPTER_RADIO_ENABLED || CONFIG_BICOPTER_CRSF_RADIO_ENABLED)

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
#elif CONFIG_BICOPTER_CRSF_RADIO_ENABLED
// Real hardware init path. Reviewed against ESP-IDF's UART driver API, not exercised: no physical
// CRSF RC receiver was available in this environment, and CONFIG_BICOPTER_CRSF_RADIO_ENABLED
// defaults to "n" for exactly that reason (see docs/radio.md's verified-vs-deferred section).
static esp_err_t radio_hw_init(crsf_radio_handle_t *out_handle)
{
    crsf_radio_config_t cfg = {
        .uart_port = (uart_port_t)CONFIG_BICOPTER_CRSF_UART_PORT,
        .rx_gpio = CONFIG_BICOPTER_CRSF_UART_RX_GPIO,
        .tx_gpio = CONFIG_BICOPTER_CRSF_UART_TX_GPIO,
        .baud_rate = CONFIG_BICOPTER_CRSF_BAUD_RATE,
        .command_timeout_us = (int64_t)CONFIG_BICOPTER_CRSF_COMMAND_TIMEOUT_MS * 1000,
        .nominal_frame_period_us = CONFIG_BICOPTER_CRSF_NOMINAL_FRAME_PERIOD_US,
        .channel_map = {
            .throttle_channel = CONFIG_BICOPTER_CRSF_THROTTLE_CHANNEL,
            .roll_channel = CONFIG_BICOPTER_CRSF_ROLL_CHANNEL,
            .pitch_channel = CONFIG_BICOPTER_CRSF_PITCH_CHANNEL,
            .yaw_channel = CONFIG_BICOPTER_CRSF_YAW_CHANNEL,
            .arm_channel = CONFIG_BICOPTER_CRSF_ARM_CHANNEL,
            .flight_mode_channel = CONFIG_BICOPTER_CRSF_FLIGHT_MODE_CHANNEL,
            .raw_min = CONFIG_BICOPTER_CRSF_RAW_MIN,
            .raw_center = CONFIG_BICOPTER_CRSF_RAW_CENTER,
            .raw_max = CONFIG_BICOPTER_CRSF_RAW_MAX,
            .arm_threshold = CONFIG_BICOPTER_CRSF_ARM_THRESHOLD,
            .flight_mode_positions = CONFIG_BICOPTER_CRSF_FLIGHT_MODE_POSITIONS,
        },
        .event_queue_length = 8,
    };

    return crsf_radio_init(&cfg, out_handle);
}
#endif

void radio_task(void *pvParameters)
{
    (void)pvParameters;

    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

#if RADIO_TRANSPORT_ENABLED
#if CONFIG_BICOPTER_RADIO_ENABLED
    esp_now_radio_handle_t transport_handle = NULL;
#elif CONFIG_BICOPTER_CRSF_RADIO_ENABLED
    crsf_radio_handle_t transport_handle = NULL;
#endif
    radio_t radio = { 0 };
    bool radio_ready = false;

    if (radio_hw_init(&transport_handle) == ESP_OK) {
#if CONFIG_BICOPTER_RADIO_ENABLED
        radio = esp_now_radio_as_radio(transport_handle);
        ESP_LOGI(TAG, "ESP-NOW radio initialized (channel=%d, timeout=%dms)",
                 CONFIG_BICOPTER_RADIO_WIFI_CHANNEL, CONFIG_BICOPTER_RADIO_COMMAND_TIMEOUT_MS);
#elif CONFIG_BICOPTER_CRSF_RADIO_ENABLED
        radio = crsf_radio_as_radio(transport_handle);
        ESP_LOGI(TAG, "CRSF radio initialized (uart=%d, baud=%d, timeout=%dms)",
                 CONFIG_BICOPTER_CRSF_UART_PORT, CONFIG_BICOPTER_CRSF_BAUD_RATE,
                 CONFIG_BICOPTER_CRSF_COMMAND_TIMEOUT_MS);
#endif
        radio_ready = true;
    } else {
        ESP_LOGW(TAG, "radio init failed; running task loop without a radio transport");
    }
#else
    ESP_LOGI(TAG,
             "no radio transport enabled (see docs/radio.md); running the task/watchdog loop "
             "without real radio I/O");
#endif

    uint32_t cycle = 0;
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        cycle++;

#if RADIO_TRANSPORT_ENABLED
        if (radio_ready) {
            size_t processed = 0;
#if CONFIG_BICOPTER_RADIO_ENABLED
            esp_now_radio_process_pending(transport_handle, &processed);
#elif CONFIG_BICOPTER_CRSF_RADIO_ENABLED
            crsf_radio_process_pending(transport_handle, &processed);
#endif
        }
#endif

        // Milestone 16: publish the latest command/health for SafetyTask's real arming/failsafe
        // evaluation (radio_state.h) - every cycle, not decimated, so SafetyTask (100Hz) never
        // reads a value staler than one RadioTask period (20ms).
        {
            radio_state_t pub = {0};
#if RADIO_TRANSPORT_ENABLED
            pub.available = radio_ready;
            if (radio_ready) {
                pub.has_command = radio_has_command(&radio);
                if (pub.has_command) {
                    radio_get_command(&radio, &pub.command);
                }
                radio_get_health(&radio, &pub.health);
            }
#endif
            radio_state_set(&pub);
        }

        if ((cycle % 50) == 1) { // ~once/second at 50Hz
#if RADIO_TRANSPORT_ENABLED
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
            ESP_LOGI(TAG, "cycle=%lu (no radio transport enabled)", (unsigned long)cycle);
#endif
        }

        esp_task_wdt_reset();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(RADIO_TASK_PERIOD_MS));
    }
}
