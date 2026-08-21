// Real automated tests for battery ADC-to-voltage/percentage/threshold conversion math
// (battery_convert.c). Same hand-rolled harness as the rest of this project's host tests.
#include <stdio.h>
#include <string.h>

#include "battery_convert.h"

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

#define CHECK_NEAR(a, b, tol, msg) CHECK(((a) - (b) <= (tol)) && ((b) - (a) <= (tol)), msg)

static void test_adc_to_voltage(void)
{
    // 1000mV raw at a 10:1 divider -> 10.0V pack voltage.
    CHECK_NEAR(battery_adc_to_voltage(1000, 10.0f), 10.0f, 1e-4f, "1000mV @ 10:1 divider == 10.0V");

    // 3300mV raw at a 1:1 (no divider) ratio -> 3.3V.
    CHECK_NEAR(battery_adc_to_voltage(3300, 1.0f), 3.3f, 1e-4f, "3300mV @ 1:1 == 3.3V");

    CHECK(battery_adc_to_voltage(1000, 0.0f) == 0.0f, "a zero divider ratio returns 0, not a crash");
    CHECK(battery_adc_to_voltage(1000, -1.0f) == 0.0f,
          "a negative divider ratio returns 0, not a negative voltage");
}

static void test_adc_to_current(void)
{
    // 1000mV raw at 40A/V -> 40A.
    CHECK_NEAR(battery_adc_to_current(1000, 40.0f), 40.0f, 1e-4f, "1000mV @ 40A/V == 40A");
    CHECK(battery_adc_to_current(1000, 0.0f) == 0.0f, "a zero scale returns 0 (current sensing off)");
}

static void test_voltage_to_percent_interpolation(void)
{
    static const battery_curve_point_t curve[] = {
        {3.0f, 0.0f},
        {3.5f, 50.0f},
        {4.0f, 100.0f},
    };
    size_t len = sizeof(curve) / sizeof(curve[0]);

    CHECK_NEAR(battery_voltage_to_percent(3.0f, curve, len), 0.0f, 1e-4f, "exact first point");
    CHECK_NEAR(battery_voltage_to_percent(3.5f, curve, len), 50.0f, 1e-4f, "exact middle point");
    CHECK_NEAR(battery_voltage_to_percent(4.0f, curve, len), 100.0f, 1e-4f, "exact last point");
    CHECK_NEAR(battery_voltage_to_percent(3.25f, curve, len), 25.0f, 1e-4f,
               "linear interpolation halfway between the first two points");
    CHECK_NEAR(battery_voltage_to_percent(3.75f, curve, len), 75.0f, 1e-4f,
               "linear interpolation halfway between the last two points");
}

static void test_voltage_to_percent_clamps_outside_range(void)
{
    static const battery_curve_point_t curve[] = {
        {3.0f, 0.0f},
        {4.2f, 100.0f},
    };
    size_t len = sizeof(curve) / sizeof(curve[0]);

    CHECK_NEAR(battery_voltage_to_percent(2.0f, curve, len), 0.0f, 1e-4f,
               "voltage below the table's range clamps to the first point's percent");
    CHECK_NEAR(battery_voltage_to_percent(5.0f, curve, len), 100.0f, 1e-4f,
               "voltage above the table's range clamps to the last point's percent");
}

static void test_voltage_to_percent_empty_curve(void)
{
    CHECK(battery_voltage_to_percent(3.7f, NULL, 0) == 0.0f, "a NULL curve returns 0");

    static const battery_curve_point_t one_point[] = {{3.7f, 42.0f}};
    CHECK_NEAR(battery_voltage_to_percent(3.0f, one_point, 1), 42.0f, 1e-4f,
               "a single-point curve returns that point's percent for any voltage");
    CHECK_NEAR(battery_voltage_to_percent(4.5f, one_point, 1), 42.0f, 1e-4f,
               "a single-point curve returns that point's percent for any voltage (above too)");
}

static void test_default_single_cell_curve_is_monotonic_and_bounded(void)
{
    size_t len = 0;
    const battery_curve_point_t *curve = battery_default_single_cell_curve(&len);
    CHECK(curve != NULL && len > 1, "the default curve is non-empty");

    for (size_t i = 1; i < len; i++) {
        CHECK(curve[i].voltage_v > curve[i - 1].voltage_v, "the default curve is ascending by voltage");
        CHECK(curve[i].percent >= curve[i - 1].percent, "the default curve's percent never decreases");
    }
    CHECK_NEAR(curve[0].percent, 0.0f, 1e-4f, "the default curve starts at 0%");
    CHECK_NEAR(curve[len - 1].percent, 100.0f, 1e-4f, "the default curve ends at 100%");
}

static void test_low_and_critical_thresholds(void)
{
    CHECK(battery_is_low(3.5f, 3.5f), "voltage exactly at the low threshold counts as low");
    CHECK(battery_is_low(3.4f, 3.5f), "voltage below the low threshold counts as low");
    CHECK(!battery_is_low(3.6f, 3.5f), "voltage above the low threshold does not count as low");

    CHECK(battery_is_critical(3.3f, 3.3f), "voltage exactly at the critical threshold counts");
    CHECK(!battery_is_critical(3.4f, 3.3f), "voltage above the critical threshold does not count");
}

static void test_convert_raw_pipeline(void)
{
    static const battery_curve_point_t curve[] = {
        {9.0f, 0.0f},
        {12.6f, 100.0f},
    };
    battery_convert_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.voltage_divider_ratio = 4.0f;
    cfg.percent_curve = curve;
    cfg.percent_curve_len = sizeof(curve) / sizeof(curve[0]);
    cfg.low_battery_voltage_v = 10.5f;
    cfg.critical_battery_voltage_v = 9.9f;
    cfg.current_sense_scale_a_per_v = 0.0f; // current sensing off

    // 3000mV raw * 4.0 ratio = 12.0V pack voltage - above low, above critical.
    battery_reading_t reading = battery_convert_raw(3000, 0, &cfg);
    CHECK_NEAR(reading.voltage_v, 12.0f, 1e-3f, "convert_raw computes the scaled pack voltage");
    CHECK(!reading.low_battery, "12.0V is above the configured low threshold");
    CHECK(!reading.critical_battery, "12.0V is above the configured critical threshold");
    CHECK(!reading.has_current, "current sensing is off (scale == 0), has_current is false");

    // 2400mV raw * 4.0 = 9.6V - below both thresholds.
    battery_reading_t low_reading = battery_convert_raw(2400, 0, &cfg);
    CHECK(low_reading.low_battery, "9.6V is at/below the low threshold");
    CHECK(low_reading.critical_battery, "9.6V is at/below the critical threshold too");

    cfg.current_sense_scale_a_per_v = 20.0f;
    battery_reading_t with_current = battery_convert_raw(3000, 500, &cfg);
    CHECK(with_current.has_current, "current sensing on (nonzero scale) reports has_current=true");
    CHECK_NEAR(with_current.current_a, 10.0f, 1e-3f, "500mV @ 20A/V == 10A");

    battery_reading_t null_cfg_reading = battery_convert_raw(0, 0, NULL);
    battery_reading_t zero_reading;
    memset(&zero_reading, 0, sizeof(zero_reading));
    CHECK(memcmp(&null_cfg_reading, &zero_reading, sizeof(battery_reading_t)) == 0,
          "a NULL config returns a zeroed reading, not a crash");
}

int main(void)
{
    test_adc_to_voltage();
    test_adc_to_current();
    test_voltage_to_percent_interpolation();
    test_voltage_to_percent_clamps_outside_range();
    test_voltage_to_percent_empty_curve();
    test_default_single_cell_curve_is_monotonic_and_bounded();
    test_low_and_critical_thresholds();
    test_convert_raw_pipeline();

    printf("%d/%d checks passed\n", g_checks - g_failures, g_checks);
    return g_failures == 0 ? 0 : 1;
}
