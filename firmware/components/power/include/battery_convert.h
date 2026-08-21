// Pure, hardware-independent battery-monitoring logic: ADC-millivolts-to-pack-voltage scaling (via
// a configurable voltage-divider ratio - no divider ratio is assumed, since no battery/divider
// hardware is chosen yet, see docs/hardware.md), an approximate voltage-to-percentage curve, and
// LOW_BATTERY/CRITICAL_BATTERY threshold checks. Nothing in this header or its .c file touches
// ADC/GPIO or any ESP-IDF header, so it compiles and runs identically on the target and on the
// host (see tests/battery_convert_test.c) - same split as
// firmware/components/sensors/include/mpu6050_convert.h (see AGENTS.md's driver-testing
// convention).
//
// Current sensing (a second ADC channel) is supported by this same conversion path - see
// battery_adc_to_current() below - but is a straight linear amps-per-volt scale, not a curve, so
// it doesn't need its own set of pure-logic types the way voltage-to-percentage does.
#ifndef BICOPTER_POWER_BATTERY_CONVERT_H
#define BICOPTER_POWER_BATTERY_CONVERT_H

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// One point on the voltage-to-percentage curve. The curve is linearly interpolated between
// consecutive points and clamped at the ends (voltage below the first point's voltage_v reads as
// that point's percent, not extrapolated below it, and likewise above the last point). This is a
// simple, explicitly configurable APPROXIMATION - real LiPo voltage-to-charge curves are famously
// nonlinear and load-dependent (rest voltage sags under load and recovers at idle, the curve's
// knee shifts with cell age/temperature/discharge rate), none of which this model attempts to
// capture. See docs/safety.md for the full caveat and the default curve's provenance (a common
// single-cell LiPo rest-voltage approximation, scaled by configured cell count).
typedef struct {
    float voltage_v;
    float percent; // 0-100
} battery_curve_point_t;

typedef struct {
    // Vbat = Vadc * voltage_divider_ratio (e.g. a 10:1 resistive divider -> 10.0f). Must be > 0;
    // battery_adc_to_voltage() returns 0 for a non-positive ratio rather than dividing/scaling by
    // a meaningless value. No specific ratio is assumed anywhere in this module - see
    // docs/hardware.md's "Battery cell count / voltage-divider and current-sense scaling: TBD"
    // entry.
    float voltage_divider_ratio;

    // Ascending-by-voltage_v curve (caller-owned, not copied by this header's functions - see
    // battery_voltage_to_percent()). NULL/zero-length disables percentage estimation
    // (battery_voltage_to_percent() then returns 0).
    const battery_curve_point_t *percent_curve;
    size_t percent_curve_len;

    float low_battery_voltage_v;
    float critical_battery_voltage_v; // should be <= low_battery_voltage_v for a sane config;
                                        // not re-validated here

    // Amps per volt at the current-sense ADC pin. 0 (the default) means current sensing is not
    // configured - see AGENTS.md/docs/safety.md: voltage monitoring is this milestone's priority,
    // current sensing is supported but not required, and this field being 0 is exactly how a
    // caller without a current-sense shunt/hall-sensor wired up opts out.
    float current_sense_scale_a_per_v;
} battery_convert_config_t;

typedef struct {
    float voltage_v;
    float percent;      // 0-100, clamped; 0 if no curve configured
    bool has_current;   // true iff current_sense_scale_a_per_v != 0
    float current_a;    // valid only when has_current
    bool low_battery;
    bool critical_battery;
} battery_reading_t;

// raw_mv is already-calibrated ADC millivolts (see battery.h - ESP-IDF's adc_cali_* API converts
// raw counts to millivolts before this pure layer ever sees a value, so no ADC-attenuation/
// resolution table lives in this ESP-IDF-free module).
float battery_adc_to_voltage(uint32_t raw_mv, float voltage_divider_ratio);

float battery_adc_to_current(uint32_t raw_mv, float current_sense_scale_a_per_v);

// Linear interpolation across `curve` (must be ascending by voltage_v; not verified here - a
// misordered curve produces a documented-undefined but non-crashing result). Clamps to
// curve[0].percent / curve[len-1].percent outside the table's range. Returns 0 for a NULL/empty
// curve.
float battery_voltage_to_percent(float voltage_v, const battery_curve_point_t *curve, size_t len);

bool battery_is_low(float voltage_v, float low_battery_voltage_v);
bool battery_is_critical(float voltage_v, float critical_battery_voltage_v);

// Convenience: runs the full raw-ADC-millivolts -> battery_reading_t pipeline in one call
// (voltage, percent, thresholds, and current if configured).
battery_reading_t battery_convert_raw(uint32_t voltage_raw_mv, uint32_t current_raw_mv,
                                       const battery_convert_config_t *cfg);

// A common single-cell (3.0V empty - 4.2V full) LiPo rest-voltage approximation - see this
// header's file comment on why it's an approximation. Scale voltage_v by cell count for a
// multi-cell pack before calling battery_voltage_to_percent() with this table, or build a
// pre-scaled table for the pack's actual configured cell count (the usual approach - see
// docs/safety.md).
const battery_curve_point_t *battery_default_single_cell_curve(size_t *out_len);

#ifdef __cplusplus
}
#endif

#endif // BICOPTER_POWER_BATTERY_CONVERT_H
