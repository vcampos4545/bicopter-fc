// Failsafe condition detection and configurable response, per docs/safety.md. Pure, ESP-IDF-free
// logic - see flight_mode.h's file header for why this lives in firmware/components/safety/, not
// flight_core/safety/.
#ifndef BICOPTER_SAFETY_FAILSAFE_H
#define BICOPTER_SAFETY_FAILSAFE_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Every condition this milestone's brief requires detected, except task-watchdog failure - the
// watchdog fires ESP-IDF's own panic/reboot path from a context application code cannot safely
// act from (see docs/safety.md's "Task watchdog" section, consistent with docs/architecture.md's
// existing finding); it is therefore not representable as an app-level failsafe_reason_t the way
// the other nine are, and is handled entirely outside this module.
typedef enum {
    FAILSAFE_REASON_NONE = 0,
    FAILSAFE_REASON_IMU_FAILURE,
    FAILSAFE_REASON_ESTIMATOR_FAILURE,
    FAILSAFE_REASON_EXCESSIVE_ATTITUDE,
    FAILSAFE_REASON_EXCESSIVE_ANGULAR_VELOCITY,
    FAILSAFE_REASON_RADIO_LOSS,
    FAILSAFE_REASON_CRITICAL_BATTERY,
    FAILSAFE_REASON_INVALID_ACTUATOR_COMMAND,
    FAILSAFE_REASON_BAROMETER_FAILURE,
    FAILSAFE_REASON_LOW_BATTERY,
    FAILSAFE_REASON_COUNT,
} failsafe_reason_t;

// Per-condition response. The design brief's default is a safe motor shutdown/disarm, never an
// attempted autonomous landing - MOTOR_SHUTDOWN is that default for every condition that
// threatens controllability. WARN_ONLY exists for the two conditions this project's own docs
// already establish don't: BAROMETER_FAILURE (the barometer is not in the attitude-control loop -
// see docs/estimation.md's altitude-estimation-deferred note) and LOW_BATTERY (a pre-critical
// warning, not yet an immediate controllability threat - forcing a shutdown from cruising
// altitude at the first LOW crossing would itself be the unsafe action; CRITICAL_BATTERY still
// defaults to MOTOR_SHUTDOWN). See docs/safety.md for the full per-condition table and rationale.
typedef enum {
    FAILSAFE_RESPONSE_MOTOR_SHUTDOWN = 0,
    FAILSAFE_RESPONSE_WARN_ONLY,
    // Documented extension point, NOT implemented by failsafe_apply_response() below (which
    // treats it identically to MOTOR_SHUTDOWN) - a future milestone building an actual
    // autonomous-landing behavior assigns this value real meaning without changing this enum's
    // other members or any existing caller's behavior.
    FAILSAFE_RESPONSE_LANDING,
} failsafe_response_t;

typedef struct {
    bool imu_valid;
    bool estimator_valid;
    bool baro_valid;
    bool radio_link_alive;

    float battery_voltage_v;

    // Angle between the current attitude and level (0 = perfectly level), radians. Only
    // meaningful when estimator_valid is true - callers must not populate this from a stale/
    // invalid estimate (see docs/safety.md's estimator-wiring caveat: no firmware caller can
    // supply a real value for this yet, since no estimator runs in firmware today).
    float tilt_angle_from_level_rad;

    // Squared magnitude of body angular rate (gx^2+gy^2+gz^2), rad^2/s^2 - squared so callers
    // never need a sqrt just to feed this struct (compared against max_angular_rate_radps^2
    // internally, same convention as arming_is_stationary()).
    float angular_rate_radps_sq;

    // Result of the defense-in-depth actuator-command check (actuator_command_check.h) for the
    // most recent command that reached this layer; true when no command has been evaluated yet
    // (nothing to reject) as well as when the most recent one passed.
    bool actuator_command_valid;
} failsafe_inputs_t;

typedef struct {
    float low_battery_voltage_v;
    float critical_battery_voltage_v;   // must be <= low_battery_voltage_v for a sane config;
                                          // failsafe_evaluate() does not re-validate this
    float max_tilt_angle_rad;            // non-positive disables the excessive-attitude check
    float max_angular_rate_radps;        // non-positive disables the excessive-rate check

    // Configurable response per condition, indexed by failsafe_reason_t (index NONE is unused).
    failsafe_response_t response[FAILSAFE_REASON_COUNT];
} failsafe_config_t;

// Default, documented config: MOTOR_SHUTDOWN for every condition except BAROMETER_FAILURE and
// LOW_BATTERY (WARN_ONLY) - see failsafe_response_t's doc above and docs/safety.md. Thresholds are
// zeroed (every threshold-based check disabled) - a caller must supply real, vehicle-specific
// values (typically from Kconfig) before use; this function only fixes the response table.
failsafe_config_t failsafe_default_config(void);

// Evaluates every condition against `in`/`cfg` and returns the single highest-priority active one
// (FAILSAFE_REASON_NONE if none), in the fixed, documented priority order implemented in
// failsafe.c (most-fundamental-sensor-failure first, so a condition whose own inputs are garbage
// because of an upstream failure never masks that failure - e.g. IMU_FAILURE outranks
// EXCESSIVE_ANGULAR_VELOCITY, since an invalid IMU sample's gyro reading is meaningless to
// threshold-check in the first place).
failsafe_reason_t failsafe_evaluate(const failsafe_inputs_t *in, const failsafe_config_t *cfg);

// Looks up the configured response for `reason`. Returns false (leaving *out_response untouched)
// for FAILSAFE_REASON_NONE, an out-of-range reason, or a NULL cfg - callers should only call this
// after failsafe_evaluate() returns non-NONE.
bool failsafe_get_response(const failsafe_config_t *cfg, failsafe_reason_t reason,
                            failsafe_response_t *out_response);

// Human-readable name, for logging/telemetry - never NULL.
const char *failsafe_reason_name(failsafe_reason_t reason);

#ifdef __cplusplus
}
#endif

#endif // BICOPTER_SAFETY_FAILSAFE_H
