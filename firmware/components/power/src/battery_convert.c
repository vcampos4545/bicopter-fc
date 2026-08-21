#include "battery_convert.h"

float battery_adc_to_voltage(uint32_t raw_mv, float voltage_divider_ratio)
{
    if (voltage_divider_ratio <= 0.0f) {
        return 0.0f;
    }
    return ((float)raw_mv / 1000.0f) * voltage_divider_ratio;
}

float battery_adc_to_current(uint32_t raw_mv, float current_sense_scale_a_per_v)
{
    if (current_sense_scale_a_per_v == 0.0f) {
        return 0.0f;
    }
    return ((float)raw_mv / 1000.0f) * current_sense_scale_a_per_v;
}

float battery_voltage_to_percent(float voltage_v, const battery_curve_point_t *curve, size_t len)
{
    if (curve == NULL || len == 0) {
        return 0.0f;
    }
    if (voltage_v <= curve[0].voltage_v) {
        return curve[0].percent;
    }
    if (voltage_v >= curve[len - 1].voltage_v) {
        return curve[len - 1].percent;
    }
    for (size_t i = 0; i + 1 < len; i++) {
        const battery_curve_point_t *lo = &curve[i];
        const battery_curve_point_t *hi = &curve[i + 1];
        if (voltage_v >= lo->voltage_v && voltage_v <= hi->voltage_v) {
            float span = hi->voltage_v - lo->voltage_v;
            if (span <= 0.0f) {
                return lo->percent;
            }
            float t = (voltage_v - lo->voltage_v) / span;
            return lo->percent + t * (hi->percent - lo->percent);
        }
    }
    // Unreachable given the ascending-curve precondition and the range checks above; fall back to
    // the last point rather than an uninitialized read.
    return curve[len - 1].percent;
}

bool battery_is_low(float voltage_v, float low_battery_voltage_v)
{
    return voltage_v <= low_battery_voltage_v;
}

bool battery_is_critical(float voltage_v, float critical_battery_voltage_v)
{
    return voltage_v <= critical_battery_voltage_v;
}

battery_reading_t battery_convert_raw(uint32_t voltage_raw_mv, uint32_t current_raw_mv,
                                       const battery_convert_config_t *cfg)
{
    battery_reading_t out = {0};
    if (cfg == NULL) {
        return out;
    }

    out.voltage_v = battery_adc_to_voltage(voltage_raw_mv, cfg->voltage_divider_ratio);
    out.percent = battery_voltage_to_percent(out.voltage_v, cfg->percent_curve,
                                              cfg->percent_curve_len);
    out.low_battery = battery_is_low(out.voltage_v, cfg->low_battery_voltage_v);
    out.critical_battery = battery_is_critical(out.voltage_v, cfg->critical_battery_voltage_v);

    if (cfg->current_sense_scale_a_per_v != 0.0f) {
        out.has_current = true;
        out.current_a = battery_adc_to_current(current_raw_mv, cfg->current_sense_scale_a_per_v);
    }

    return out;
}

const battery_curve_point_t *battery_default_single_cell_curve(size_t *out_len)
{
    // A widely-used rough single-cell LiPo rest-voltage/state-of-charge approximation - NOT
    // manufacturer data, NOT load-corrected. See battery_convert.h's file header and
    // docs/safety.md for the caveat this is meant to carry at every call site.
    static const battery_curve_point_t kCurve[] = {
        {3.00f, 0.0f},  {3.40f, 5.0f},   {3.60f, 10.0f},  {3.70f, 20.0f},
        {3.75f, 30.0f}, {3.80f, 40.0f},  {3.85f, 50.0f},  {3.90f, 60.0f},
        {3.95f, 70.0f}, {4.00f, 80.0f},  {4.10f, 90.0f},  {4.20f, 100.0f},
    };
    if (out_len != NULL) {
        *out_len = sizeof(kCurve) / sizeof(kCurve[0]);
    }
    return kCurve;
}
