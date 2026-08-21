// Generic Radio HAL shape (docs/architecture.md: `send(telemetry packet)` / `receive() ->
// setpoint packet, or none`, refined here into a poll-style vtable matching this project's other
// HAL interfaces -- see firmware/components/actuators/include/motor_output.h for the pattern this
// follows: a small C "vtable" (ops struct + opaque ctx pointer) so a second protocol
// implementation (Milestone 15's conventional RC receiver) is a new pair of files populating its
// own `radio_ops_t` and handing out a `radio_t` the same way `esp_now_radio_as_radio()` does --
// any code holding a `radio_t` never changes when that lands. `esp_now_radio.c/.h` (this
// milestone) is the only implementation so far; this header is the seam, not the implementation.
// No ESP-NOW-specific type appears anywhere in this file.
//
// This is deliberately a local, plain-C shape (like imu.h/motor_output.h), not flight_core's
// eventual interface definition -- flight_core does not yet have a Radio interface of its own
// (see AGENTS.md).
#ifndef BICOPTER_RADIO_RADIO_H
#define BICOPTER_RADIO_RADIO_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// One decoded command from the pilot/ground-station side, in the units flight_core will
// eventually consume directly -- no protocol-native units (raw packet bytes, PWM microseconds,
// etc.) survive past the implementation's parsing step. Stick axes are normalized, not yet mapped
// to a physical rate/angle setpoint -- that mapping is flight mode logic (docs/architecture.md's
// data-flow step), a `flight_core`/`FlightControlTask` concern this interface deliberately stays
// out of.
typedef struct {
    float throttle;        // [0, 1]
    float roll;             // [-1, 1], stick-normalized
    float pitch;            // [-1, 1], stick-normalized
    float yaw;               // [-1, 1], stick-normalized
    bool arm;                // pilot arm switch/command
    uint8_t flight_mode;      // raw mode index; semantics owned by future flight-mode logic, not
                               // this HAL layer
    uint32_t sequence;         // the packet's sequence number, for callers that want it (e.g.
                               // telemetry echo); staleness/reordering is already handled before
                               // a command reaches here -- see radio_has_command()/health below
    int64_t timestamp_us;      // sender-side timestamp carried in the packet (informational)
} radio_command_t;

// Radio-link health, refreshed on every get_health() call using the implementation's own
// notion of "now" -- callers never compute staleness themselves from a raw timestamp (see
// AGENTS.md Milestone 14 note: this milestone's job is making link loss reliably *detectable*,
// not deciding what to do about it -- that's Milestone 16).
typedef struct {
    float packet_loss_percent;         // 0-100, estimated from sequence-number gaps
    int64_t last_valid_rx_timestamp_us; // implementation's monotonic clock; 0 if never received
    bool link_alive;                     // false if no valid command within the configured
                                          // staleness timeout (or none ever received)
    bool rssi_available;                 // false if the transport/hardware doesn't expose RSSI
    int8_t rssi_dbm;                     // valid only when rssi_available is true
} radio_health_t;

typedef struct {
    // True if the link currently has a usable command (a valid packet was received within the
    // configured staleness timeout). Never blocks.
    bool (*has_command)(void *ctx);

    // Fills `out_command` with the most recently accepted command. Returns ESP_ERR_NOT_FOUND if
    // no valid command has ever been received (out_command is left zeroed in that case); callers
    // that only care about freshness should check has_command()/get_health() first, since a
    // command can be returned successfully here even if the link has since gone stale -- this
    // call reports the last-known value, not link liveness.
    esp_err_t (*get_command)(void *ctx, radio_command_t *out_command);

    // Fills `out_health` with current link-health metrics (see radio_health_t).
    esp_err_t (*get_health)(void *ctx, radio_health_t *out_health);

    // Releases any resources (Wi-Fi/radio peripheral, queues, mutex) held by ctx.
    esp_err_t (*deinit)(void *ctx);
} radio_ops_t;

// One Radio instance: an ops vtable plus the opaque implementation state it operates on. Built by
// an implementation's `*_as_radio()` function (e.g. esp_now_radio_as_radio()) -- never
// constructed by hand outside that implementation.
typedef struct {
    const radio_ops_t *ops;
    void *ctx;
} radio_t;

static inline bool radio_has_command(radio_t *radio)
{
    return radio->ops->has_command(radio->ctx);
}

static inline esp_err_t radio_get_command(radio_t *radio, radio_command_t *out_command)
{
    return radio->ops->get_command(radio->ctx, out_command);
}

static inline esp_err_t radio_get_health(radio_t *radio, radio_health_t *out_health)
{
    return radio->ops->get_health(radio->ctx, out_health);
}

static inline esp_err_t radio_deinit(radio_t *radio)
{
    return radio->ops->deinit(radio->ctx);
}

#ifdef __cplusplus
}
#endif

#endif // BICOPTER_RADIO_RADIO_H
