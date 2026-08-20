// Pure, hardware-independent tilt-servo-PWM logic: angle clamping and the angle-to-pulse-width
// mapping. Nothing in this header or its .c file touches LEDC, GPIO, or any ESP-IDF header, so it
// compiles and runs identically on the target and on the host (see tests/servo_convert_test.c) -
// same split as mpu6050_convert.h/bmp581_convert.h (see AGENTS.md "Driver testing convention").
#ifndef BICOPTER_ACTUATORS_SERVO_CONVERT_H
#define BICOPTER_ACTUATORS_SERVO_CONVERT_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Everything needed to interpret a ServoOutput angle command (radians, servo-frame - see
// servo_output.h) as a pulse width, for one tilt servo. `min_pulse_us`/`max_pulse_us` are this
// servo's calibrated endpoints - `min_angle_rad` maps to `min_pulse_us`, `max_angle_rad` maps to
// `max_pulse_us`. See docs/hardware.md for the tilt-servo geometry/range assumptions.
typedef struct {
    float min_angle_rad;      // minimum commandable tilt angle
    float max_angle_rad;      // maximum commandable tilt angle
    float neutral_angle_rad;  // angle commanded at safe-init and by *_set_neutral(); must lie
                                // within [min_angle_rad, max_angle_rad]
    uint32_t min_pulse_us;    // pulse width at min_angle_rad
    uint32_t max_pulse_us;    // pulse width at max_angle_rad
    bool invert_direction;    // mirrors the angle-to-pulse mapping around the configured range,
                                // for a servo mounted with its horn reversed relative to the
                                // other tilt unit (see docs/hardware.md)
} servo_convert_config_t;

// Clamps `angle_rad` to [min(min_angle_rad,max_angle_rad), max(min_angle_rad,max_angle_rad)] (the
// min/max are sorted defensively in case of a swapped config). Non-finite input (NaN/Inf) clamps
// to `neutral_angle_rad`. Returns the clamped value; *out_was_clamped is set true whenever the
// input was not already exactly equal to the output, so a caller (e.g. servo_output_write()) can
// log out-of-range commands instead of silently absorbing them (see AGENTS.md milestone-5 brief:
// never accept an out-of-range value silently).
float servo_clamp_angle(float angle_rad, const servo_convert_config_t *config,
                         bool *out_was_clamped);

// Maps an angle to a pulse width in microseconds, applying min/max_pulse_us and
// invert_direction. Also clamps its input to the configured angle range internally as a safety
// net - callers that want the "was this out of range" signal for logging should still call
// servo_clamp_angle first. If min_angle_rad == max_angle_rad (degenerate config), always returns
// min_pulse_us.
uint32_t servo_angle_to_pulse_us(float angle_rad, const servo_convert_config_t *config);

#ifdef __cplusplus
}
#endif

#endif // BICOPTER_ACTUATORS_SERVO_CONVERT_H
