#include "pwm_esc_convert.h"

#include <math.h>
#include <stddef.h>

#include "pwm_util.h"

float pwm_esc_clamp_throttle(float throttle, const pwm_esc_convert_config_t *config,
                              bool *out_was_clamped)
{
    float lo = pwm_clampf(config->min_throttle, 0.0f, 1.0f);
    float hi = pwm_clampf(config->max_throttle, 0.0f, 1.0f);
    if (hi < lo) {
        float tmp = lo;
        lo = hi;
        hi = tmp;
    }

    bool clamped = false;
    float value;
    if (isnan(throttle) || isinf(throttle)) {
        value = lo;
        clamped = true;
    } else {
        value = throttle;
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

uint32_t pwm_esc_throttle_to_pulse_us(float throttle, const pwm_esc_convert_config_t *config)
{
    float t = isnan(throttle) || isinf(throttle) ? 0.0f : pwm_clampf(throttle, 0.0f, 1.0f);
    if (config->invert_direction) {
        t = 1.0f - t;
    }

    float span = (float)config->max_pulse_us - (float)config->min_pulse_us;
    float pulse = (float)config->min_pulse_us + t * span;
    return (uint32_t)(pulse + 0.5f);
}
