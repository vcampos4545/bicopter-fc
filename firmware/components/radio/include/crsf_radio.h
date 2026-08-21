// CRSF (Crossfire) RC-receiver radio driver -- the second concrete implementation of this
// project's protocol-independent Radio HAL interface (radio.h), alongside Milestone 14's
// esp_now_radio.c/.h. No radio.h changes were needed: this implementation populates every
// existing radio_command_t/radio_health_t field using the exact same interface ESP-NOW already
// implements -- see docs/radio.md for the two fields (sequence, timestamp_us) CRSF fills
// differently (a local counter and a receiver-side timestamp, since the CRSF wire format carries
// neither) and why that's a data-availability difference, not an interface gap.
//
// UART driver context (see docs/architecture.md "Interrupt context vs. task context" and
// docs/radio.md for the full writeup): unlike Milestone 14's ESP-NOW driver, this driver doesn't
// write its own ISR or receive callback at all. ESP-IDF's UART driver (uart_driver_install())
// already does that internally -- a driver-owned interrupt copies bytes from the UART's hardware
// FIFO into a driver-owned RX ring buffer and posts uart_event_t notifications (UART_DATA,
// UART_FIFO_OVF, ...) to the QueueHandle_t the driver hands back at install time. That interrupt
// and the byte copy it does are ESP-IDF-internal, not this project's code, unlike the MPU6050
// driver's GPIO ISR (mpu6050.c) or even ESP-NOW's task-context receive callback
// (esp_now_radio.c) -- both of which this project wrote and had to keep minimal by hand. All of
// *this* driver's own logic -- draining the event queue, pulling bytes out of the ring buffer via
// uart_read_bytes(), byte-level frame sync, CRC/type validation, channel decoding, and updating
// the state radio_t's ops read -- runs in crsf_radio_process_pending(), called once per
// RadioTask cycle (firmware/main/radio_task.c), the same "dedicated task, not a new one" pattern
// Milestone 14 established.
//
// Receive-only this milestone: CRSF telemetry write-back (this device -> receiver -> transmitter)
// is real protocol capability but out of scope here -- see crsf_frame.h's file header for the
// full scope-narrowing rationale (only RC_CHANNELS_PACKED is parsed).
#ifndef BICOPTER_RADIO_CRSF_RADIO_H
#define BICOPTER_RADIO_CRSF_RADIO_H

#include <stddef.h>
#include <stdint.h>

#include "driver/uart.h"
#include "esp_err.h"

#include "crsf_frame.h"
#include "radio.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct crsf_radio_dev *crsf_radio_handle_t;

typedef struct {
    uart_port_t uart_port;
    int rx_gpio; // receiver's UART TX pin connects here
    int tx_gpio; // UART_PIN_NO_CHANGE (-1) if unused; unused this milestone (receive-only)
    int baud_rate; // CRSF's common default is 420000; some links use 400000/921600/higher, so
                    // this is configured, not hardcoded to one value
    int64_t command_timeout_us; // same has_command()/get_health().link_alive semantics as
                                  // esp_now_radio_config_t.command_timeout_us
    int64_t nominal_frame_period_us; // for the packet-loss estimate; see crsf_frame.h
    crsf_channel_map_t channel_map;
    size_t event_queue_length; // depth of the UART driver's own event queue; 0 selects a default
} crsf_radio_config_t;

// Configures the UART peripheral (8N1, non-inverted -- CRSF, unlike SBUS, needs no signal
// inversion) and installs ESP-IDF's UART driver in interrupt/ring-buffer mode (not busy-polling).
// Reviewed against ESP-IDF's UART driver API; not exercised on real hardware -- see docs/radio.md.
esp_err_t crsf_radio_init(const crsf_radio_config_t *config, crsf_radio_handle_t *out_handle);

// Drains the UART driver's event queue and does all real work -- see this header's file comment
// for the full ISR/driver/task-context split. Call on every RadioTask cycle (or any other
// dedicated task's cadence). `out_frames_processed` (optional) receives the number of complete,
// successfully-validated RC_CHANNELS_PACKED frames decoded this call.
esp_err_t crsf_radio_process_pending(crsf_radio_handle_t handle, size_t *out_frames_processed);

// Wraps `handle` in the generic radio_t HAL interface (radio.h), mirroring
// esp_now_radio_as_radio().
radio_t crsf_radio_as_radio(crsf_radio_handle_t handle);

// Uninstalls the UART driver and releases the mutex/handle.
esp_err_t crsf_radio_deinit(crsf_radio_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif // BICOPTER_RADIO_CRSF_RADIO_H
