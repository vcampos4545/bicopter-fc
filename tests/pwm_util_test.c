// Real automated tests for the shared PWM pure logic (pwm_util.c): float clamping (used by
// pwm_esc_convert.c and servo_convert.c) and pulse-width-in-microseconds-to-LEDC-duty-count
// conversion (used by pwm_esc_output.c and servo_output.c). Same hand-rolled harness as
// tests/mpu6050_convert_test.c (see that file for why).

#include <stdio.h>

#include "pwm_util.h"

static int g_failures = 0;
static int g_checks = 0;

#define CHECK(cond, msg)                                                                       \
    do {                                                                                        \
        g_checks++;                                                                             \
        if (!(cond)) {                                                                          \
            g_failures++;                                                                       \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, msg);                                \
        }                                                                                        \
    } while (0)

#define CHECK_NEAR_U32(a, b, tol, msg)                                                          \
    CHECK(((a) > (b) ? (a) - (b) : (b) - (a)) <= (uint32_t)(tol), msg)

static void test_clampf_within_range_unchanged(void)
{
    CHECK(pwm_clampf(0.5f, 0.0f, 1.0f) == 0.5f, "in-range value passes through unchanged");
}

static void test_clampf_below_and_above_range(void)
{
    CHECK(pwm_clampf(-1.0f, 0.0f, 1.0f) == 0.0f, "below-floor value clamps to lo");
    CHECK(pwm_clampf(2.0f, 0.0f, 1.0f) == 1.0f, "above-ceiling value clamps to hi");
}

static void test_clampf_boundary_values(void)
{
    CHECK(pwm_clampf(0.0f, 0.0f, 1.0f) == 0.0f, "value exactly at lo passes through");
    CHECK(pwm_clampf(1.0f, 0.0f, 1.0f) == 1.0f, "value exactly at hi passes through");
}

static void test_zero_pulse_is_zero_duty(void)
{
    CHECK(pwm_pulse_us_to_duty(0, 50, 16) == 0, "zero pulse width -> zero duty");
}

static void test_full_period_is_max_duty(void)
{
    // 50Hz -> 20000us period. A pulse width equal to the full period should saturate at the
    // maximum representable duty count for the given resolution.
    uint32_t max_duty = (1u << 16) - 1u;
    CHECK(pwm_pulse_us_to_duty(20000, 50, 16) == max_duty,
          "pulse width == full period -> max duty count");
}

static void test_pulse_wider_than_period_clamps(void)
{
    uint32_t max_duty = (1u << 16) - 1u;
    CHECK(pwm_pulse_us_to_duty(50000, 50, 16) == max_duty,
          "pulse width wider than the PWM period clamps to max duty, does not wrap");
}

static void test_half_period_is_roughly_half_duty(void)
{
    uint32_t max_duty = (1u << 16) - 1u;
    CHECK_NEAR_U32(pwm_pulse_us_to_duty(10000, 50, 16), max_duty / 2, 2,
                   "half the PWM period is roughly half the max duty count");
}

static void test_standard_rc_pwm_endpoints_50hz_16bit(void)
{
    // 50Hz period = 20000us. 1000us/2000us are the standard RC PWM pulse-width endpoints.
    CHECK_NEAR_U32(pwm_pulse_us_to_duty(1000, 50, 16), 3277, 2,
                   "1000us pulse @ 50Hz/16-bit ~= 3277 duty counts (1000/20000 * 65535)");
    CHECK_NEAR_U32(pwm_pulse_us_to_duty(1500, 50, 16), 4915, 2,
                   "1500us pulse @ 50Hz/16-bit ~= 4915 duty counts (1500/20000 * 65535)");
    CHECK_NEAR_U32(pwm_pulse_us_to_duty(2000, 50, 16), 6554, 2,
                   "2000us pulse @ 50Hz/16-bit ~= 6554 duty counts (2000/20000 * 65535)");
}

static void test_invalid_freq_or_resolution_returns_zero(void)
{
    CHECK(pwm_pulse_us_to_duty(1500, 0, 16) == 0, "zero freq_hz returns 0, not a divide-by-zero result");
    CHECK(pwm_pulse_us_to_duty(1500, 50, 0) == 0, "zero duty_resolution_bits returns 0");
    CHECK(pwm_pulse_us_to_duty(1500, 50, 32) == 0, "duty_resolution_bits above the defensive cap returns 0");
}

static void test_higher_resolution_scales_duty_up(void)
{
    // Same pulse/period, higher resolution should give a proportionally larger duty count.
    uint32_t duty_16bit = pwm_pulse_us_to_duty(1500, 50, 16);
    uint32_t duty_10bit = pwm_pulse_us_to_duty(1500, 50, 10);
    CHECK(duty_16bit > duty_10bit, "higher duty_resolution_bits yields a larger duty count for the same pulse");
}

int main(void)
{
    test_clampf_within_range_unchanged();
    test_clampf_below_and_above_range();
    test_clampf_boundary_values();
    test_zero_pulse_is_zero_duty();
    test_full_period_is_max_duty();
    test_pulse_wider_than_period_clamps();
    test_half_period_is_roughly_half_duty();
    test_standard_rc_pwm_endpoints_50hz_16bit();
    test_invalid_freq_or_resolution_returns_zero();
    test_higher_resolution_scales_duty_up();

    printf("%d/%d checks passed\n", g_checks - g_failures, g_checks);
    return g_failures == 0 ? 0 : 1;
}
