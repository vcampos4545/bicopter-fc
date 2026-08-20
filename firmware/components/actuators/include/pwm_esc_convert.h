// Pure, hardware-independent ESC-PWM logic: normalized-throttle clamping and the
// throttle-to-pulse-width mapping. Nothing in this header or its .c file touches LEDC, GPIO, or
// any ESP-IDF header, so it compiles and runs identically on the target and on the host (see
// tests/pwm_esc_convert_test.c) - same split as mpu6050_convert.h/bmp581_convert.h (see
// AGENTS.md "Driver testing convention").
#ifndef BICOPTER_ACTUATORS_PWM_ESC_CONVERT_H
#define BICOPTER_ACTUATORS_PWM_ESC_CONVERT_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Everything needed to interpret a normalized MotorOutput throttle command as a pulse width, for
// one ESC channel. `min_pulse_us`/`max_pulse_us` are this ESC's calibrated endpoints (see
// docs/hardware.md's ESC calibration section for how those are normally determined on real
// hardware) - throttle 0.0 maps to `min_pulse_us`, 1.0 maps to `max_pulse_us`.
typedef struct {
    float min_throttle;      // normalized floor clamp applied to requested throttle, in [0,1]
    float max_throttle;      // normalized ceiling clamp applied to requested throttle, in [0,1]
    uint32_t min_pulse_us;   // pulse width at throttle == 0.0 (ESC's calibrated low endpoint)
    uint32_t max_pulse_us;   // pulse width at throttle == 1.0 (ESC's calibrated high endpoint)
    uint32_t idle_pulse_us;  // pulse width commanded at safe-init and by *_set_idle(), independent
                              // of min_pulse_us so a conservative idle value can be set even before
                              // min_throttle/max_throttle calibration is finalized
    bool invert_direction;   // swaps which pulse endpoint throttle 0.0/1.0 map to; see
                              // docs/hardware.md for when this is (and is not) the right way to
                              // correct a per-unit wiring/rotation difference
} pwm_esc_convert_config_t;

// Clamps `throttle` to [config->min_throttle, config->max_throttle] (themselves clamped to
// [0,1] first, and swapped if min > max, so a malformed config can't invert the whole range).
// Non-finite input (NaN/Inf) clamps to the floor. Returns the clamped value; *out_was_clamped is
// set true whenever the input was not already exactly equal to the output, so a caller (e.g.
// pwm_esc_output_write()) can log out-of-range commands instead of silently absorbing them (see
// AGENTS.md milestone-5 brief: never accept an out-of-range value silently).
float pwm_esc_clamp_throttle(float throttle, const pwm_esc_convert_config_t *config,
                              bool *out_was_clamped);

// Maps a normalized throttle to a pulse width in microseconds, applying min/max_pulse_us and
// invert_direction. Also clamps its input to [0,1] internally as a safety net (a caller that
// skips pwm_esc_clamp_throttle can still never command a pulse width outside
// [min_pulse_us, max_pulse_us]) - callers that want the "was this out of range" signal for
// logging should still call pwm_esc_clamp_throttle first.
uint32_t pwm_esc_throttle_to_pulse_us(float throttle, const pwm_esc_convert_config_t *config);

#ifdef __cplusplus
}
#endif

#endif // BICOPTER_ACTUATORS_PWM_ESC_CONVERT_H
