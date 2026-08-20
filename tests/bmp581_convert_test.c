// Real automated tests for the BMP581 driver's pure logic: raw-register-to-SI conversion,
// pressure filtering, pressure-to-altitude derivation, and stale/invalid-data detection
// (bmp581_convert.c). Same hand-rolled assert-and-report harness as
// tests/mpu6050_convert_test.c (no external test framework is available offline in this
// environment), exiting non-zero on any failure so it works as a real CI gate.
//
// What this file verifies vs. what remains unverified until real hardware is available: this
// covers the math/logic only (register scaling, filter behavior, altitude formula, validity
// flagging, staleness comparison). It does NOT exercise I2C transactions or actual sensor
// registers — those require real silicon, which was not available for this milestone. See
// docs/hardware.md for what's deferred.

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "bmp581_convert.h"

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

static void test_sign_extend_24(void)
{
    CHECK(bmp581_sign_extend_24(0x000000) == 0, "zero stays zero");
    CHECK(bmp581_sign_extend_24(0x000001) == 1, "positive value unchanged");
    CHECK(bmp581_sign_extend_24(0x7FFFFF) == 8388607, "largest positive 24-bit value");
    CHECK(bmp581_sign_extend_24(0xFFFFFF) == -1, "all-ones 24-bit == -1");
    CHECK(bmp581_sign_extend_24(0x800000) == -8388608, "sign bit alone == most negative value");
}

static void test_temperature_conversion(void)
{
    // LSB = 1/65536 degC.
    CHECK_NEAR(bmp581_convert_temperature_c(0), 0.0f, 1e-6f, "zero raw == 0 degC");
    CHECK_NEAR(bmp581_convert_temperature_c(65536), 1.0f, 1e-6f, "65536 raw == 1 degC");
    CHECK_NEAR(bmp581_convert_temperature_c(25 * 65536), 25.0f, 1e-3f, "25 degC round-trips");
    CHECK_NEAR(bmp581_convert_temperature_c(-65536), -1.0f, 1e-6f, "negative raw == negative degC");
}

static void test_pressure_conversion(void)
{
    // LSB = 1/64 Pa.
    CHECK_NEAR(bmp581_convert_pressure_pa(0), 0.0f, 1e-6f, "zero raw == 0 Pa");
    CHECK_NEAR(bmp581_convert_pressure_pa(64), 1.0f, 1e-6f, "64 raw == 1 Pa");
    CHECK_NEAR(bmp581_convert_pressure_pa(101325u * 64u), 101325.0f, 1e-1f,
               "101325 Pa round-trips");
}

static void test_filter_seeds_from_first_sample(void)
{
    bmp581_filter_state_t state;
    memset(&state, 0, sizeof(state));

    float out = bmp581_filter_apply(&state, 0.1f, 100000.0f);
    CHECK_NEAR(out, 100000.0f, 1e-6f, "first sample seeds filter directly, no ramp-up");
    CHECK(state.initialized, "filter marked initialized after first sample");
}

static void test_filter_alpha_one_is_pass_through(void)
{
    bmp581_filter_state_t state;
    memset(&state, 0, sizeof(state));

    bmp581_filter_apply(&state, 1.0f, 100000.0f);
    float out = bmp581_filter_apply(&state, 1.0f, 100500.0f);
    CHECK_NEAR(out, 100500.0f, 1e-6f, "alpha=1.0 passes each new sample through unchanged");
}

static void test_filter_smooths_with_low_alpha(void)
{
    bmp581_filter_state_t state;
    memset(&state, 0, sizeof(state));

    bmp581_filter_apply(&state, 0.1f, 100000.0f);
    float out = bmp581_filter_apply(&state, 0.1f, 101000.0f);
    // 0.1*101000 + 0.9*100000 = 100100
    CHECK_NEAR(out, 100100.0f, 1e-2f, "low alpha weights new sample only 10%%");
    CHECK(out < 101000.0f && out > 100000.0f, "filtered value lands strictly between old and new");
}

static void test_filter_clamps_out_of_range_alpha(void)
{
    bmp581_filter_state_t state;
    memset(&state, 0, sizeof(state));
    bmp581_filter_apply(&state, 0.5f, 100000.0f);

    float out_over = bmp581_filter_apply(&state, 5.0f, 100500.0f);
    CHECK_NEAR(out_over, 100500.0f, 1e-6f, "alpha > 1 clamped to 1 (pass-through)");

    bmp581_filter_state_t state2;
    memset(&state2, 0, sizeof(state2));
    bmp581_filter_apply(&state2, 0.5f, 100000.0f);
    float out_under = bmp581_filter_apply(&state2, -1.0f, 999999.0f);
    CHECK_NEAR(out_under, 100000.0f, 1e-6f, "alpha < 0 clamped to 0 (ignores new sample)");
}

static void test_altitude_at_reference_pressure_is_zero(void)
{
    float alt = bmp581_pressure_to_altitude_m(101325.0f, 101325.0f);
    CHECK_NEAR(alt, 0.0f, 1e-3f, "pressure == sea_level_pa gives altitude 0");
}

static void test_altitude_increases_as_pressure_drops(void)
{
    float alt_low = bmp581_pressure_to_altitude_m(100000.0f, 101325.0f);
    float alt_high = bmp581_pressure_to_altitude_m(95000.0f, 101325.0f);
    CHECK(alt_low > 0.0f, "pressure below sea-level reference gives positive altitude");
    CHECK(alt_high > alt_low, "lower pressure gives higher derived altitude");
    // Standard atmosphere: ~111m for a 1325 Pa drop from 101325 Pa near sea level.
    CHECK_NEAR(alt_low, 111.0f, 5.0f, "~111m for 100000 Pa vs. 101325 Pa reference");
}

static void test_altitude_negative_above_reference_pressure(void)
{
    // Pressure higher than the reference (e.g. QNH set for a different day) gives negative
    // altitude, i.e. "below" the reference.
    float alt = bmp581_pressure_to_altitude_m(102000.0f, 101325.0f);
    CHECK(alt < 0.0f, "pressure above sea_level_pa gives negative altitude");
}

static void test_raw_to_reading_valid(void)
{
    bmp581_convert_config_t config = { .sea_level_pa = 101325.0f, .filter_alpha = 1.0f };
    bmp581_filter_state_t filter_state;
    memset(&filter_state, 0, sizeof(filter_state));

    bmp581_raw_sample_t raw = {
        .temperature_raw = 25 * 65536,
        .pressure_raw = 101325u * 64u,
    };

    barometer_reading_t reading = bmp581_raw_to_reading(raw, &config, &filter_state, 12345, true);

    CHECK(reading.valid, "reading valid when i2c_ok");
    CHECK(reading.timestamp_us == 12345, "timestamp passed through unchanged");
    CHECK_NEAR(reading.temperature_c, 25.0f, 1e-3f, "temperature converted from raw");
    CHECK_NEAR(reading.pressure_pa, 101325.0f, 1e-1f, "pressure converted from raw");
    CHECK_NEAR(reading.altitude_m, 0.0f, 1e-2f, "altitude derived from converted pressure");
}

static void test_raw_to_reading_invalid_on_i2c_failure(void)
{
    bmp581_convert_config_t config = { .sea_level_pa = 101325.0f, .filter_alpha = 1.0f };
    bmp581_filter_state_t filter_state;
    memset(&filter_state, 0, sizeof(filter_state));

    bmp581_raw_sample_t raw = { .temperature_raw = 25 * 65536, .pressure_raw = 101325u * 64u };

    barometer_reading_t reading =
        bmp581_raw_to_reading(raw, &config, &filter_state, 999, /*i2c_ok=*/false);

    CHECK(!reading.valid, "reading invalid when I2C transaction failed");
    CHECK_NEAR(reading.pressure_pa, 0.0f, 1e-6f, "pressure zeroed, not garbage, on I2C failure");
    CHECK_NEAR(reading.altitude_m, 0.0f, 1e-6f, "altitude zeroed, not garbage, on I2C failure");
    CHECK(reading.timestamp_us == 999, "timestamp still set on an invalid reading");
    CHECK(!filter_state.initialized, "a failed read must not seed/perturb the filter state");
}

static void test_raw_to_reading_failed_read_does_not_perturb_filter(void)
{
    bmp581_convert_config_t config = { .sea_level_pa = 101325.0f, .filter_alpha = 0.1f };
    bmp581_filter_state_t filter_state;
    memset(&filter_state, 0, sizeof(filter_state));

    bmp581_raw_sample_t good_raw = { .temperature_raw = 0, .pressure_raw = 100000u * 64u };
    barometer_reading_t first = bmp581_raw_to_reading(good_raw, &config, &filter_state, 1, true);
    CHECK_NEAR(first.pressure_pa, 100000.0f, 1e-1f, "first valid reading seeds filter");

    bmp581_raw_sample_t bad_raw = { .temperature_raw = 0, .pressure_raw = 999999u * 64u };
    bmp581_raw_to_reading(bad_raw, &config, &filter_state, 2, /*i2c_ok=*/false);

    CHECK_NEAR(filter_state.filtered_pressure_pa, 100000.0f, 1e-1f,
               "filter state unchanged after a failed read");
}

static void test_staleness(void)
{
    CHECK(!bmp581_is_stale(1000, 1500, 1000), "not stale when age < max_age");
    CHECK(!bmp581_is_stale(1000, 2000, 1000), "not stale exactly at max_age boundary");
    CHECK(bmp581_is_stale(1000, 2001, 1000), "stale when age > max_age");
    CHECK(!bmp581_is_stale(1000, 1000, 1000), "not stale with zero elapsed time");
}

int main(void)
{
    test_sign_extend_24();
    test_temperature_conversion();
    test_pressure_conversion();
    test_filter_seeds_from_first_sample();
    test_filter_alpha_one_is_pass_through();
    test_filter_smooths_with_low_alpha();
    test_filter_clamps_out_of_range_alpha();
    test_altitude_at_reference_pressure_is_zero();
    test_altitude_increases_as_pressure_drops();
    test_altitude_negative_above_reference_pressure();
    test_raw_to_reading_valid();
    test_raw_to_reading_invalid_on_i2c_failure();
    test_raw_to_reading_failed_read_does_not_perturb_filter();
    test_staleness();

    printf("%d/%d checks passed\n", g_checks - g_failures, g_checks);
    return g_failures == 0 ? 0 : 1;
}
