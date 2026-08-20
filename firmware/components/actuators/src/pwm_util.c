#include "pwm_util.h"

float pwm_clampf(float v, float lo, float hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

uint32_t pwm_pulse_us_to_duty(uint32_t pulse_us, uint32_t freq_hz, uint32_t duty_resolution_bits)
{
    if (freq_hz == 0 || duty_resolution_bits == 0 || duty_resolution_bits > 31) {
        return 0;
    }

    double period_us = 1e6 / (double)freq_hz;
    double pulse = (double)pulse_us;
    if (pulse > period_us) {
        pulse = period_us;
    }

    uint32_t max_duty = (1u << duty_resolution_bits) - 1u;
    double duty = (pulse / period_us) * (double)max_duty;
    return (uint32_t)(duty + 0.5);
}
