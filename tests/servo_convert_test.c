// Real automated tests for the ServoOutput driver's pure logic: angle clamping and the
// angle-to-pulse-width mapping (servo_convert.c). Same hand-rolled harness as
// tests/mpu6050_convert_test.c (see that file for why).
//
// What this covers vs. what remains unverified until real hardware is available: this covers the
// clamping/mapping math only. It does NOT exercise LEDC peripheral configuration or an actual PWM
// signal on a GPIO, or real servo mechanical range - see docs/hardware.md for what's deferred.

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "servo_convert.h"

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

// -pi/4 .. +pi/4 tilt range, 1000..2000us pulse, neutral at 0 rad (1500us).
static servo_convert_config_t default_config(void)
{
    servo_convert_config_t config;
    memset(&config, 0, sizeof(config));
    config.min_angle_rad = -(float)M_PI / 4.0f;
    config.max_angle_rad = (float)M_PI / 4.0f;
    config.neutral_angle_rad = 0.0f;
    config.min_pulse_us = 1000;
    config.max_pulse_us = 2000;
    config.invert_direction = false;
    return config;
}

static void test_clamp_in_range(void)
{
    servo_convert_config_t config = default_config();
    bool was_clamped = true;

    float value = servo_clamp_angle(0.1f, &config, &was_clamped);
    CHECK_NEAR(value, 0.1f, 1e-6f, "in-range angle passes through unchanged");
    CHECK(!was_clamped, "in-range angle not reported as clamped");
}

static void test_clamp_below_min(void)
{
    servo_convert_config_t config = default_config();
    bool was_clamped = false;

    float value = servo_clamp_angle(-10.0f, &config, &was_clamped);
    CHECK_NEAR(value, config.min_angle_rad, 1e-6f, "below-min angle clamps to min_angle_rad");
    CHECK(was_clamped, "below-min angle reported as clamped");
}

static void test_clamp_above_max(void)
{
    servo_convert_config_t config = default_config();
    bool was_clamped = false;

    float value = servo_clamp_angle(10.0f, &config, &was_clamped);
    CHECK_NEAR(value, config.max_angle_rad, 1e-6f, "above-max angle clamps to max_angle_rad");
    CHECK(was_clamped, "above-max angle reported as clamped");
}

static void test_clamp_nonfinite_goes_to_neutral(void)
{
    servo_convert_config_t config = default_config();
    config.neutral_angle_rad = 0.05f;
    bool was_clamped = false;

    float nan_result = servo_clamp_angle(NAN, &config, &was_clamped);
    CHECK_NEAR(nan_result, 0.05f, 1e-6f, "NaN angle clamps to neutral_angle_rad");
    CHECK(was_clamped, "NaN angle reported as clamped");
}

static void test_clamp_swapped_min_max_config(void)
{
    servo_convert_config_t config = default_config();
    config.min_angle_rad = 1.0f;
    config.max_angle_rad = -1.0f; // malformed: max < min
    bool was_clamped = false;

    float value = servo_clamp_angle(0.0f, &config, &was_clamped);
    CHECK(value >= -1.0f && value <= 1.0f, "swapped min/max config still produces an in-range result");
}

static void test_angle_to_pulse_endpoints(void)
{
    servo_convert_config_t config = default_config();
    CHECK(servo_angle_to_pulse_us(config.min_angle_rad, &config) == 1000,
          "min_angle_rad -> min_pulse_us");
    CHECK(servo_angle_to_pulse_us(config.max_angle_rad, &config) == 2000,
          "max_angle_rad -> max_pulse_us");
    CHECK(servo_angle_to_pulse_us(0.0f, &config) == 1500, "neutral (midpoint) angle -> midpoint pulse");
}

static void test_angle_to_pulse_invert_direction(void)
{
    servo_convert_config_t config = default_config();
    config.invert_direction = true;

    CHECK(servo_angle_to_pulse_us(config.min_angle_rad, &config) == 2000,
          "inverted: min_angle_rad -> max_pulse_us");
    CHECK(servo_angle_to_pulse_us(config.max_angle_rad, &config) == 1000,
          "inverted: max_angle_rad -> min_pulse_us");
}

static void test_angle_to_pulse_clamps_out_of_range_input(void)
{
    servo_convert_config_t config = default_config();
    CHECK(servo_angle_to_pulse_us(-10.0f, &config) == 1000,
          "unclamped far-below-range angle input still floors at min_pulse_us");
    CHECK(servo_angle_to_pulse_us(10.0f, &config) == 2000,
          "unclamped far-above-range angle input still ceils at max_pulse_us");
}

static void test_angle_to_pulse_degenerate_zero_span(void)
{
    servo_convert_config_t config = default_config();
    config.min_angle_rad = 0.2f;
    config.max_angle_rad = 0.2f; // degenerate: zero-width range

    CHECK(servo_angle_to_pulse_us(0.2f, &config) == 1000,
          "degenerate zero-width angle range returns min_pulse_us, not garbage/NaN/div-by-zero");
}

int main(void)
{
    test_clamp_in_range();
    test_clamp_below_min();
    test_clamp_above_max();
    test_clamp_nonfinite_goes_to_neutral();
    test_clamp_swapped_min_max_config();
    test_angle_to_pulse_endpoints();
    test_angle_to_pulse_invert_direction();
    test_angle_to_pulse_clamps_out_of_range_input();
    test_angle_to_pulse_degenerate_zero_span();

    printf("%d/%d checks passed\n", g_checks - g_failures, g_checks);
    return g_failures == 0 ? 0 : 1;
}
