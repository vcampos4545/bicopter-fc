#include "bmp581_convert.h"

#include <math.h>
#include <string.h>

int32_t bmp581_sign_extend_24(uint32_t raw24)
{
    if (raw24 & 0x00800000u) {
        return (int32_t)(raw24 | 0xFF000000u);
    }
    return (int32_t)raw24;
}

float bmp581_convert_temperature_c(int32_t raw_temp)
{
    return (float)raw_temp / 65536.0f;
}

float bmp581_convert_pressure_pa(uint32_t raw_pressure)
{
    return (float)raw_pressure / 64.0f;
}

float bmp581_filter_apply(bmp581_filter_state_t *state, float alpha, float raw_pressure_pa)
{
    if (!state->initialized) {
        state->filtered_pressure_pa = raw_pressure_pa;
        state->initialized = true;
        return state->filtered_pressure_pa;
    }

    float a = alpha;
    if (a < 0.0f) {
        a = 0.0f;
    } else if (a > 1.0f) {
        a = 1.0f;
    }

    state->filtered_pressure_pa = a * raw_pressure_pa + (1.0f - a) * state->filtered_pressure_pa;
    return state->filtered_pressure_pa;
}

float bmp581_pressure_to_altitude_m(float pressure_pa, float sea_level_pa)
{
    return 44330.0f * (1.0f - powf(pressure_pa / sea_level_pa, 0.190284f));
}

barometer_reading_t bmp581_raw_to_reading(bmp581_raw_sample_t raw,
                                           const bmp581_convert_config_t *config,
                                           bmp581_filter_state_t *filter_state,
                                           int64_t timestamp_us,
                                           bool i2c_ok)
{
    barometer_reading_t reading;
    memset(&reading, 0, sizeof(reading));
    reading.timestamp_us = timestamp_us;
    reading.valid = i2c_ok;

    if (!reading.valid) {
        return reading;
    }

    float raw_pressure_pa = bmp581_convert_pressure_pa(raw.pressure_raw);
    reading.temperature_c = bmp581_convert_temperature_c(raw.temperature_raw);
    reading.pressure_pa = bmp581_filter_apply(filter_state, config->filter_alpha, raw_pressure_pa);
    reading.altitude_m = bmp581_pressure_to_altitude_m(reading.pressure_pa, config->sea_level_pa);

    return reading;
}

bool bmp581_is_stale(int64_t last_valid_timestamp_us, int64_t now_us, int64_t max_age_us)
{
    return (now_us - last_valid_timestamp_us) > max_age_us;
}
