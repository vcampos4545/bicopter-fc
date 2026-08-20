#include "servo_convert.h"

#include <math.h>
#include <stddef.h>

#include "pwm_util.h"

static void sorted_range(const servo_convert_config_t *config, float *out_lo, float *out_hi)
{
    float lo = config->min_angle_rad;
    float hi = config->max_angle_rad;
    if (hi < lo) {
        float tmp = lo;
        lo = hi;
        hi = tmp;
    }
    *out_lo = lo;
    *out_hi = hi;
}

float servo_clamp_angle(float angle_rad, const servo_convert_config_t *config,
                         bool *out_was_clamped)
{
    float lo, hi;
    sorted_range(config, &lo, &hi);

    bool clamped = false;
    float value;
    if (isnan(angle_rad) || isinf(angle_rad)) {
        value = config->neutral_angle_rad;
        clamped = true;
    } else {
        value = angle_rad;
        if (value < lo) {
            value = lo;
            clamped = true;
        } else if (value > hi) {
            value = hi;
            clamped = true;
        }
    }

    if (out_was_clamped != NULL) {
        *out_was_clamped = clamped;
    }
    return value;
}

uint32_t servo_angle_to_pulse_us(float angle_rad, const servo_convert_config_t *config)
{
    float lo, hi;
    sorted_range(config, &lo, &hi);

    float span_angle = hi - lo;
    float frac;
    if (fabsf(span_angle) < 1e-9f) {
        frac = 0.0f;
    } else {
        float a = isnan(angle_rad) || isinf(angle_rad) ? lo : pwm_clampf(angle_rad, lo, hi);
        frac = (a - lo) / span_angle;
    }
    if (config->invert_direction) {
        frac = 1.0f - frac;
    }

    float span_pulse = (float)config->max_pulse_us - (float)config->min_pulse_us;
    float pulse = (float)config->min_pulse_us + frac * span_pulse;
    return (uint32_t)(pulse + 0.5f);
}
