#include "esp_now_radio.h"

#include <stdlib.h>
#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "nvs_flash.h"

static const char *TAG = "esp_now_radio";

// One raw ESP-NOW payload as handed off by the receive callback -- see esp_now_recv_cb() below
// for why this copy is the *only* work the callback does.
typedef struct {
    uint8_t data[ESP_NOW_MAX_DATA_LEN];
    uint8_t len;
    int8_t rssi;
    bool rssi_valid;
} raw_packet_t;

struct esp_now_radio_dev {
    uint8_t peer_mac[6];
    int64_t command_timeout_us;

    QueueHandle_t rx_queue; // holds raw_packet_t, filled by esp_now_recv_cb()

    // Everything below is read by radio_t's ops (called from any task) and written by
    // esp_now_radio_process_pending() (called from RadioTask) -- mutex-protected for the same
    // "multi-field state must be observed consistently" reason as firmware/main/safety_state.c.
    SemaphoreHandle_t mutex;
    bool has_command;
    radio_command_t last_command;
    radio_loss_tracker_t loss_tracker;
    int64_t last_valid_rx_timestamp_us;
    int8_t last_rssi_dbm;
    bool last_rssi_valid;
};

// Only one Wi-Fi/ESP-NOW peripheral exists on this chip, and esp_now_register_recv_cb() takes a
// bare function pointer with no per-instance context -- see esp_now_radio.h's file header. This
// back-pointer is how the free-function callback below reaches instance state.
static struct esp_now_radio_dev *s_active_dev;

// --- ESP-NOW callback: Wi-Fi driver task context, not ISR, but still foreign/latency-sensitive
// context this driver must not do real work in -- see esp_now_radio.h's file header and
// docs/radio.md. Bounds-check the length, copy the bytes (and RSSI, if the driver exposed it) --
// nothing else. No parsing, no logging, no blocking calls; xQueueSend with a zero timeout drops
// the packet on a full queue rather than ever blocking the Wi-Fi task.
static void esp_now_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    struct esp_now_radio_dev *dev = s_active_dev;
    if (dev == NULL || data == NULL || len <= 0 || (size_t)len > ESP_NOW_MAX_DATA_LEN) {
        return;
    }

    raw_packet_t raw;
    raw.len = (uint8_t)len;
    memcpy(raw.data, data, (size_t)len);
    if (info != NULL && info->rx_ctrl != NULL) {
        raw.rssi = info->rx_ctrl->rssi;
        raw.rssi_valid = true;
    } else {
        raw.rssi = 0;
        raw.rssi_valid = false;
    }

    xQueueSend(dev->rx_queue, &raw, 0);
}

esp_err_t esp_now_radio_init(const esp_now_radio_config_t *config, esp_now_radio_handle_t *out_handle)
{
    if (config == NULL || out_handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_active_dev != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    struct esp_now_radio_dev *dev = calloc(1, sizeof(*dev));
    if (dev == NULL) {
        return ESP_ERR_NO_MEM;
    }
    memcpy(dev->peer_mac, config->peer_mac, sizeof(dev->peer_mac));
    dev->command_timeout_us = config->command_timeout_us;
    radio_loss_tracker_reset(&dev->loss_tracker);

    dev->mutex = xSemaphoreCreateMutex();
    size_t queue_len = (config->rx_queue_length > 0) ? config->rx_queue_length : 4;
    dev->rx_queue = xQueueCreate(queue_len, sizeof(raw_packet_t));
    if (dev->mutex == NULL || dev->rx_queue == NULL) {
        if (dev->mutex != NULL) {
            vSemaphoreDelete(dev->mutex);
        }
        if (dev->rx_queue != NULL) {
            vQueueDelete(dev->rx_queue);
        }
        free(dev);
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        err = nvs_flash_erase();
        if (err == ESP_OK) {
            err = nvs_flash_init();
        }
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_flash_init failed: %s", esp_err_to_name(err));
        goto fail_no_wifi;
    }

    err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) { // INVALID_STATE = already initialized
        ESP_LOGE(TAG, "esp_netif_init failed: %s", esp_err_to_name(err));
        goto fail_no_wifi;
    }
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "esp_event_loop_create_default failed: %s", esp_err_to_name(err));
        goto fail_no_wifi;
    }

    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&wifi_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_init failed: %s", esp_err_to_name(err));
        goto fail_no_wifi;
    }
    // ESP-NOW doesn't need Wi-Fi credentials persisted to flash; keep them in RAM only.
    err = esp_wifi_set_storage(WIFI_STORAGE_RAM);
    if (err != ESP_OK) {
        goto fail_wifi;
    }
    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK) {
        goto fail_wifi;
    }
    err = esp_wifi_start();
    if (err != ESP_OK) {
        goto fail_wifi;
    }
    err = esp_wifi_set_channel(config->wifi_channel, WIFI_SECOND_CHAN_NONE);
    if (err != ESP_OK) {
        goto fail_wifi;
    }

    err = esp_now_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_now_init failed: %s", esp_err_to_name(err));
        goto fail_wifi;
    }

    s_active_dev = dev; // must be set before registering the callback, which reads it
    err = esp_now_register_recv_cb(esp_now_recv_cb);
    if (err != ESP_OK) {
        s_active_dev = NULL;
        goto fail_espnow;
    }

    esp_now_peer_info_t peer = { 0 };
    memcpy(peer.peer_addr, config->peer_mac, sizeof(peer.peer_addr));
    peer.channel = config->wifi_channel;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    err = esp_now_add_peer(&peer);
    if (err != ESP_OK && err != ESP_ERR_ESPNOW_EXIST) {
        ESP_LOGE(TAG, "esp_now_add_peer failed: %s", esp_err_to_name(err));
        esp_now_unregister_recv_cb();
        s_active_dev = NULL;
        goto fail_espnow;
    }

    *out_handle = dev;
    return ESP_OK;

fail_espnow:
    esp_now_deinit();
fail_wifi:
    esp_wifi_stop();
    esp_wifi_deinit();
fail_no_wifi:
    vQueueDelete(dev->rx_queue);
    vSemaphoreDelete(dev->mutex);
    free(dev);
    return err;
}

esp_err_t esp_now_radio_process_pending(esp_now_radio_handle_t handle, size_t *out_processed_count)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t processed = 0;
    raw_packet_t raw;
    while (xQueueReceive(handle->rx_queue, &raw, 0) == pdTRUE) {
        processed++;

        radio_command_packet_t parsed;
        if (!radio_command_packet_parse(raw.data, raw.len, &parsed)) {
            ESP_LOGW(TAG, "rejected malformed/undersized packet (len=%u)", raw.len);
            continue;
        }

        xSemaphoreTake(handle->mutex, portMAX_DELAY);
        bool accept = radio_seq_is_newer(parsed.sequence, handle->loss_tracker.last_sequence,
                                          handle->loss_tracker.has_last_sequence);
        if (accept) {
            radio_loss_tracker_record(&handle->loss_tracker, parsed.sequence);
            handle->last_command = (radio_command_t){
                .throttle = parsed.throttle,
                .roll = parsed.roll,
                .pitch = parsed.pitch,
                .yaw = parsed.yaw,
                .arm = parsed.arm,
                .flight_mode = parsed.flight_mode,
                .sequence = parsed.sequence,
                .timestamp_us = parsed.timestamp_us,
            };
            handle->has_command = true;
            handle->last_valid_rx_timestamp_us = esp_timer_get_time();
            handle->last_rssi_dbm = raw.rssi;
            handle->last_rssi_valid = raw.rssi_valid;
        }
        xSemaphoreGive(handle->mutex);

        if (!accept) {
            ESP_LOGD(TAG, "rejected stale/reordered/duplicate packet seq=%lu",
                     (unsigned long)parsed.sequence);
        }
    }

    if (out_processed_count != NULL) {
        *out_processed_count = processed;
    }
    return ESP_OK;
}

esp_err_t esp_now_radio_send_telemetry(esp_now_radio_handle_t handle,
                                        const radio_telemetry_packet_t *telemetry)
{
    if (handle == NULL || telemetry == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t buf[RADIO_TELEMETRY_PACKET_WIRE_SIZE];
    size_t n = radio_telemetry_packet_serialize(telemetry, buf, sizeof(buf));
    if (n == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    return esp_now_send(handle->peer_mac, buf, n);
}

// --- radio_t ops: read/write the mutex-protected state above; never block waiting on ESP-NOW
// itself (get_command/get_health/has_command are always non-blocking, per radio.h's contract).

static bool esp_now_radio_op_has_command(void *ctx)
{
    struct esp_now_radio_dev *dev = (struct esp_now_radio_dev *)ctx;
    int64_t now_us = esp_timer_get_time();

    xSemaphoreTake(dev->mutex, portMAX_DELAY);
    bool has = dev->has_command &&
               !radio_is_stale(dev->last_valid_rx_timestamp_us, now_us, dev->command_timeout_us);
    xSemaphoreGive(dev->mutex);
    return has;
}

static esp_err_t esp_now_radio_op_get_command(void *ctx, radio_command_t *out_command)
{
    struct esp_now_radio_dev *dev = (struct esp_now_radio_dev *)ctx;
    if (out_command == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(dev->mutex, portMAX_DELAY);
    bool has = dev->has_command;
    *out_command = dev->last_command;
    xSemaphoreGive(dev->mutex);

    return has ? ESP_OK : ESP_ERR_NOT_FOUND;
}

static esp_err_t esp_now_radio_op_get_health(void *ctx, radio_health_t *out_health)
{
    struct esp_now_radio_dev *dev = (struct esp_now_radio_dev *)ctx;
    if (out_health == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    int64_t now_us = esp_timer_get_time();

    xSemaphoreTake(dev->mutex, portMAX_DELAY);
    out_health->packet_loss_percent = radio_loss_tracker_loss_percent(&dev->loss_tracker);
    out_health->last_valid_rx_timestamp_us = dev->last_valid_rx_timestamp_us;
    out_health->link_alive =
        dev->has_command && !radio_is_stale(dev->last_valid_rx_timestamp_us, now_us,
                                             dev->command_timeout_us);
    out_health->rssi_available = dev->last_rssi_valid;
    out_health->rssi_dbm = dev->last_rssi_dbm;
    xSemaphoreGive(dev->mutex);

    return ESP_OK;
}

esp_err_t esp_now_radio_deinit(esp_now_radio_handle_t handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_now_unregister_recv_cb();
    esp_now_del_peer(handle->peer_mac);
    esp_now_deinit();
    esp_wifi_stop();
    esp_wifi_deinit();

    if (s_active_dev == handle) {
        s_active_dev = NULL;
    }
    vQueueDelete(handle->rx_queue);
    vSemaphoreDelete(handle->mutex);
    free(handle);
    return ESP_OK;
}

static esp_err_t esp_now_radio_op_deinit(void *ctx)
{
    return esp_now_radio_deinit((esp_now_radio_handle_t)ctx);
}

static const radio_ops_t s_esp_now_radio_ops = {
    .has_command = esp_now_radio_op_has_command,
    .get_command = esp_now_radio_op_get_command,
    .get_health = esp_now_radio_op_get_health,
    .deinit = esp_now_radio_op_deinit,
};

radio_t esp_now_radio_as_radio(esp_now_radio_handle_t handle)
{
    return (radio_t){ .ops = &s_esp_now_radio_ops, .ctx = handle };
}
