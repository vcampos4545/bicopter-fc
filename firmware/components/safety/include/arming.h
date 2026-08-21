// Arming-precondition logic: every documented gate DISARMED -> ARMED must pass before the flight-
// mode state machine (flight_mode.h) allows arming. Pure, ESP-IDF-free logic - see flight_mode.h's
// file header for why this lives in firmware/components/safety/, not flight_core/safety/.
#ifndef BICOPTER_SAFETY_ARMING_H
#define BICOPTER_SAFETY_ARMING_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// One bit per precondition, so a caller (SafetyTask's log line, future telemetry) can report
// *which* precondition(s) are currently blocking arming, not just a single pass/fail bit.
typedef enum {
    ARMING_BLOCK_IMU              = 1u << 0,
    ARMING_BLOCK_ESTIMATOR        = 1u << 1,
    ARMING_BLOCK_RADIO_LINK       = 1u << 2,
    ARMING_BLOCK_RADIO_ARM_CMD    = 1u << 3,
    ARMING_BLOCK_BATTERY          = 1u << 4,
    ARMING_BLOCK_CRITICAL_ERROR   = 1u << 5,
    ARMING_BLOCK_NOT_STATIONARY   = 1u << 6,
} arming_block_flag_t;

typedef struct {
    bool imu_valid;           // Milestone 3's imu_reading_t.valid (not stale/invalid)
    bool estimator_valid;     // Milestone 8's AttitudeEstimate::valid
    bool radio_link_alive;    // Milestone 14/15's radio_health_t.link_alive
    bool radio_arm_command;   // radio_command_t.arm - explicit pilot arm request, required in
                               // addition to link_alive (a live but never-armed link must not
                               // arm on its own)
    bool battery_ok;          // battery voltage above the configured LOW_BATTERY threshold
                               // (battery_convert.h) - a pack already at/below LOW must not begin
                               // a flight, even though LOW_BATTERY's in-flight failsafe response
                               // is a warning, not a forced landing (see docs/safety.md)
    bool no_critical_errors;  // caller-supplied rollup: true iff no non-battery/non-radio/non-
                               // sensor critical condition is active (e.g. an invalid actuator
                               // command already latched, or any future condition a caller wants
                               // to gate arming on without adding a new named field here)
    bool is_stationary;       // arming_is_stationary() below, evaluated on the current gyro sample
} arming_preconditions_t;

// True only when every precondition holds. Never partially arms - this is a single all-or-nothing
// gate, per the design brief's "must never auto-arm" requirement: a caller must supply real,
// current values for every field, not assume a missing signal defaults to "ok."
bool arming_preconditions_met(const arming_preconditions_t *p);

// Bitmask (arming_block_flag_t) of every currently-failing precondition. Zero iff
// arming_preconditions_met() would return true for the same input.
uint32_t arming_blocking_mask(const arming_preconditions_t *p);

// True if the vehicle's angular rate is below `max_angular_rate_radps` on every axis, evaluated
// via squared magnitude (no sqrt needed: compares gx^2+gy^2+gz^2 against the threshold squared).
// Guards against arming while the vehicle is being handled/moved - the design brief's explicit
// "check the vehicle is stationary during arming" requirement. A non-positive threshold disables
// the check (always considered stationary), matching this project's other "non-positive disables"
// convention (see e.g. AttitudeControllerConfig::rate_limit_radps in flight_core/control).
bool arming_is_stationary(float gyro_x_radps, float gyro_y_radps, float gyro_z_radps,
                           float max_angular_rate_radps);

#ifdef __cplusplus
}
#endif

#endif // BICOPTER_SAFETY_ARMING_H
