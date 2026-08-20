// Pure PWM math shared by pwm_esc_convert.c and servo_convert.c (float clamping) and by
// pwm_esc_output.c/servo_output.c (pulse-width-to-LEDC-duty-count conversion). No ESP-IDF
// dependency, so it is host-tested directly (tests/pwm_util_test.c) rather than only exercised
// via hardware - same split as the sensors component's *_convert.c/.h modules (see AGENTS.md
// "Driver testing convention").
#ifndef BICOPTER_ACTUATORS_PWM_UTIL_H
#define BICOPTER_ACTUATORS_PWM_UTIL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Clamps `v` to [lo, hi]. Shared by pwm_esc_convert.c and servo_convert.c so their clamping
// behavior can't silently drift apart between the two otherwise-parallel modules.
float pwm_clampf(float v, float lo, float hi);

// Converts a pulse width in microseconds to an LEDC duty count, given the channel's configured
// PWM frequency and duty resolution (in bits, e.g. 16 for LEDC_TIMER_16_BIT). `pulse_us` is
// clamped to [0, period_us] (period_us = 1e6/freq_hz) before conversion, so a pulse width wider
// than one PWM period can never wrap or overflow the returned duty count. Returns 0 for an
// invalid `freq_hz` (0) or `duty_resolution_bits` (0, or greater than 31 - LEDC's own maximum is
// well under this, but this function stays defensive rather than assuming a specific SoC's cap).
uint32_t pwm_pulse_us_to_duty(uint32_t pulse_us, uint32_t freq_hz, uint32_t duty_resolution_bits);

#ifdef __cplusplus
}
#endif

#endif // BICOPTER_ACTUATORS_PWM_UTIL_H
