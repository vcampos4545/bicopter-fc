// ESP-NOW radio driver -- the first concrete implementation of this project's protocol-independent
// Radio HAL interface (radio.h). Public API deliberately keeps every ESP-NOW-specific type
// (esp_now_peer_info_t, wifi_*, ...) out of radio.h; only this header and esp_now_radio.c know
// ESP-NOW exists.
//
// Callback/task-context split (see docs/architecture.md "Interrupt context vs. task context" and
// docs/radio.md for the full writeup): ESP-NOW's receive callback runs in the Wi-Fi driver's own
// task context, NOT interrupt context (unlike the MPU6050 driver's genuine GPIO ISR -- see
// mpu6050.c). It is still foreign, high-priority, latency-sensitive context this driver must not
// block or do real work in: esp_now_recv_cb() (esp_now_radio.c) does exactly one thing --
// bounds-check the length and copy the raw bytes (plus RSSI, if available) onto a FreeRTOS queue
// via a non-blocking xQueueSend(). No packet parsing, no logging, no blocking calls happen there.
// All real work -- wire-format validation, sequence-number staleness/reordering rejection,
// packet-loss tracking, and updating the state radio_t's ops read -- happens in
// esp_now_radio_process_pending(), which must be called from a dedicated task on a regular
// cadence; this milestone wires that call into Milestone 6's existing RadioTask
// (firmware/main/radio_task.c), per the design brief.
//
// Only one esp_now_radio instance may exist at a time: ESP-NOW's receive callback
// (esp_now_register_recv_cb()) takes a bare function pointer with no per-instance context
// argument, and there is only one Wi-Fi/ESP-NOW peripheral on this chip anyway, so this isn't a
// real limitation -- esp_now_radio_init() returns ESP_ERR_INVALID_STATE if called again before a
// prior instance is deinitialized.
#ifndef BICOPTER_RADIO_ESP_NOW_RADIO_H
#define BICOPTER_RADIO_ESP_NOW_RADIO_H

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#include "radio.h"
#include "radio_packet.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct esp_now_radio_dev *esp_now_radio_handle_t;

typedef struct {
    uint8_t peer_mac[6];        // paired ground-station peer's MAC address (see docs/radio.md's
                                 // pairing procedure)
    uint8_t wifi_channel;       // fixed channel both peers must agree on; 0 is not a valid
                                 // "use current" shortcut here since this device is never
                                 // associated to an AP -- pass a real channel (1-13)
    int64_t command_timeout_us; // has_command()/get_health().link_alive report the link lost if
                                 // no valid command arrives within this window (Milestone 16
                                 // decides what to *do* about that; this interface only detects it)
    size_t rx_queue_length;     // depth of the raw-packet queue between the ESP-NOW callback and
                                 // esp_now_radio_process_pending(); 0 selects a small default
} esp_now_radio_config_t;

// Brings up Wi-Fi in station mode (required for ESP-NOW, which piggybacks on the Wi-Fi radio
// without an actual AP association), initializes ESP-NOW, registers the receive callback, and
// adds `config->peer_mac` as a peer. Reviewed against ESP-IDF's ESP-NOW API docs; not exercised
// without hardware -- see docs/radio.md's verified-vs-deferred section.
esp_err_t esp_now_radio_init(const esp_now_radio_config_t *config, esp_now_radio_handle_t *out_handle);

// Drains the raw-packet queue populated by the ESP-NOW receive callback and does all real
// parsing/validation/sequence-staleness/loss-tracking work -- see this header's file comment for
// why this must run in task context, never the callback itself. Call on every RadioTask cycle (or
// any other dedicated task's cadence). `out_processed_count` (optional) receives the number of
// raw packets drained this call, whether accepted or rejected -- purely informational/for logging.
esp_err_t esp_now_radio_process_pending(esp_now_radio_handle_t handle, size_t *out_processed_count);

// Sends one telemetry packet to the paired peer -- see radio_packet.h's file comment for scope
// (a minimal downlink echo, not full vehicle-state telemetry). Safe to call from task context;
// esp_now_send() itself does not block on the actual over-the-air transmission. Not exercised
// without hardware.
esp_err_t esp_now_radio_send_telemetry(esp_now_radio_handle_t handle,
                                        const radio_telemetry_packet_t *telemetry);

// Wraps `handle` in the generic radio_t HAL interface (radio.h) so callers never depend on
// ESP-NOW specifics directly -- the seam Milestone 15's RC-receiver implementation will mirror.
radio_t esp_now_radio_as_radio(esp_now_radio_handle_t handle);

// Unregisters the receive callback, removes the peer, deinitializes ESP-NOW/Wi-Fi, and releases
// the queue/mutex/handle. Not called anywhere in this milestone's firmware/main/ wiring (RadioTask
// runs for the lifetime of the firmware); exists for interface symmetry and testability, same as
// mpu6050_deinit()/motor_output_deinit().
esp_err_t esp_now_radio_deinit(esp_now_radio_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif // BICOPTER_RADIO_ESP_NOW_RADIO_H
