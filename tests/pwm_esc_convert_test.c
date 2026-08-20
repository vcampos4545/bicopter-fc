// Real automated tests for the PwmEscOutput driver's pure logic: normalized-throttle clamping
// and the throttle-to-pulse-width mapping (pwm_esc_convert.c). Same hand-rolled harness as
// tests/mpu6050_convert_test.c (see that file for why).
//
// What this covers vs. what remains unverified until real hardware is available: this covers the
// clamping/mapping math only. It does NOT exercise LEDC peripheral configuration or an actual PWM
// signal on a GPIO - see docs/hardware.md for what's deferred.

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "pwm_esc_convert.h"

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

#define CHECK_NEAR(a, b, tol, msg) CHECK(fabsf((a) - (b)) <= (tol), msg)

static pwm_esc_convert_config_t default_config(void)
{
    pwm_esc_convert_config_t config;
    memset(&config, 0, sizeof(config));
    config.min_throttle = 0.0f;
    config.max_throttle = 1.0f;
    config.min_pulse_us = 1000;
    config.max_pulse_us = 2000;
    config.idle_pulse_us = 1000;
    config.invert_direction = false;
    return config;
}

static void test_clamp_within_range_unchanged(void)
{
    pwm_esc_convert_config_t config = default_config();
    bool was_clamped = true;

    float value = pwm_esc_clamp_throttle(0.5f, &config, &was_clamped);
    CHECK_NEAR(value, 0.5f, 1e-6f, "in-range throttle passes through unchanged");
    CHECK(!was_clamped, "in-range throttle not reported as clamped");
}

static void test_clamp_below_floor(void)
{
    pwm_esc_convert_config_t config = default_config();
    bool was_clamped = false;

    float value = pwm_esc_clamp_throttle(-0.5f, &config, &was_clamped);
    CHECK_NEAR(value, 0.0f, 1e-6f, "below-floor throttle clamps to min_throttle");
    CHECK(was_clamped, "below-floor throttle reported as clamped");
}

static void test_clamp_above_ceiling(void)
{
    pwm_esc_convert_config_t config = default_config();
    bool was_clamped = false;

    float value = pwm_esc_clamp_throttle(1.5f, &config, &was_clamped);
    CHECK_NEAR(value, 1.0f, 1e-6f, "above-ceiling throttle clamps to max_throttle");
    CHECK(was_clamped, "above-ceiling throttle reported as clamped");
}

static void test_clamp_restricted_range(void)
{
    pwm_esc_convert_config_t config = default_config();
    config.min_throttle = 0.1f;
    config.max_throttle = 0.9f;
    bool was_clamped = false;

    CHECK_NEAR(pwm_esc_clamp_throttle(0.0f, &config, &was_clamped), 0.1f, 1e-6f,
               "throttle 0.0 clamps to configured min_throttle floor");
    CHECK(was_clamped, "clamp to restricted floor reported");

    CHECK_NEAR(pwm_esc_clamp_throttle(1.0f, &config, &was_clamped), 0.9f, 1e-6f,
               "throttle 1.0 clamps to configured max_throttle ceiling");
    CHECK(was_clamped, "clamp to restricted ceiling reported");
}

static void test_clamp_nonfinite_input(void)
{
    pwm_esc_convert_config_t config = default_config();
    bool was_clamped = false;

    float nan_result = pwm_esc_clamp_throttle(NAN, &config, &was_clamped);
    CHECK_NEAR(nan_result, 0.0f, 1e-6f, "NaN throttle clamps to floor");
    CHECK(was_clamped, "NaN throttle reported as clamped");

    was_clamped = false;
    float inf_result = pwm_esc_clamp_throttle(INFINITY, &config, &was_clamped);
    CHECK_NEAR(inf_result, 0.0f, 1e-6f, "+Inf throttle clamps to floor");
    CHECK(was_clamped, "+Inf throttle reported as clamped");
}

static void test_clamp_malformed_config_swapped_min_max(void)
{
    pwm_esc_convert_config_t config = default_config();
    config.min_throttle = 0.8f;
    config.max_throttle = 0.2f; // malformed: max < min
    bool was_clamped = false;

    // The malformed config is recovered by swapping to an effective range of [0.2, 0.8], not by
    // collapsing to a single value - so an input already inside that swapped range passes through
    // unclamped, matching servo_clamp_angle()'s equivalent swap-on-malformed-config behavior.
    float in_range = pwm_esc_clamp_throttle(0.5f, &config, &was_clamped);
    CHECK_NEAR(in_range, 0.5f, 1e-6f, "swapped min/max config: input inside [0.2,0.8] passes through unchanged");
    CHECK(!was_clamped, "swapped min/max config: in-range input not reported as clamped");

    was_clamped = false;
    float above_swapped_ceiling = pwm_esc_clamp_throttle(0.9f, &config, &was_clamped);
    CHECK_NEAR(above_swapped_ceiling, 0.8f, 1e-6f,
               "swapped min/max config: input above the swapped ceiling clamps to 0.8");
    CHECK(was_clamped, "swapped min/max config: above-ceiling input reported as clamped");

    was_clamped = false;
    float below_swapped_floor = pwm_esc_clamp_throttle(0.1f, &config, &was_clamped);
    CHECK_NEAR(below_swapped_floor, 0.2f, 1e-6f,
               "swapped min/max config: input below the swapped floor clamps to 0.2");
    CHECK(was_clamped, "swapped min/max config: below-floor input reported as clamped");
}

static void test_throttle_to_pulse_endpoints(void)
{
    pwm_esc_convert_config_t config = default_config();
    CHECK(pwm_esc_throttle_to_pulse_us(0.0f, &config) == 1000, "throttle 0.0 -> min_pulse_us");
    CHECK(pwm_esc_throttle_to_pulse_us(1.0f, &config) == 2000, "throttle 1.0 -> max_pulse_us");
    CHECK(pwm_esc_throttle_to_pulse_us(0.5f, &config) == 1500, "throttle 0.5 -> midpoint pulse");
}

static void test_throttle_to_pulse_invert_direction(void)
{
    pwm_esc_convert_config_t config = default_config();
    config.invert_direction = true;

    CHECK(pwm_esc_throttle_to_pulse_us(0.0f, &config) == 2000,
          "inverted: throttle 0.0 -> max_pulse_us");
    CHECK(pwm_esc_throttle_to_pulse_us(1.0f, &config) == 1000,
          "inverted: throttle 1.0 -> min_pulse_us");
}

static void test_throttle_to_pulse_clamps_out_of_range_input(void)
{
    pwm_esc_convert_config_t config = default_config();
    CHECK(pwm_esc_throttle_to_pulse_us(-1.0f, &config) == 1000,
          "unclamped negative throttle input still floors at min_pulse_us");
    CHECK(pwm_esc_throttle_to_pulse_us(2.0f, &config) == 2000,
          "unclamped over-range throttle input still ceils at max_pulse_us");
    CHECK(pwm_esc_throttle_to_pulse_us(NAN, &config) == 1000,
          "NaN throttle input maps to min_pulse_us, not garbage");
}

int main(void)
{
    test_clamp_within_range_unchanged();
    test_clamp_below_floor();
    test_clamp_above_ceiling();
    test_clamp_restricted_range();
    test_clamp_nonfinite_input();
    test_clamp_malformed_config_swapped_min_max();
    test_throttle_to_pulse_endpoints();
    test_throttle_to_pulse_invert_direction();
    test_throttle_to_pulse_clamps_out_of_range_input();

    printf("%d/%d checks passed\n", g_checks - g_failures, g_checks);
    return g_failures == 0 ? 0 : 1;
}
