#include "pid.h"

#include <algorithm>

namespace bicopter {

namespace {
float Clamp(float value, float lo, float hi)
{
    return std::min(std::max(value, lo), hi);
}
} // namespace

Pid::Pid(const PidConfig& config) : config_(config) {}

void Pid::reset()
{
    integral_ = 0.0f;
    previous_measurement_ = 0.0f;
    previous_error_ = 0.0f;
    have_previous_ = false;
    last_unsaturated_output_ = 0.0f;
}

float Pid::update(float setpoint, float measurement, float dt)
{
    const float error = setpoint - measurement;

    if (dt <= 0.0f) {
        const float p_only = config_.kp * error;
        last_unsaturated_output_ = p_only;
        return Clamp(p_only, config_.output_min, config_.output_max);
    }

    float derivative = 0.0f;
    if (have_previous_) {
        derivative = config_.derivative_on_measurement
                         ? -(measurement - previous_measurement_) / dt
                         : (error - previous_error_) / dt;
    }

    const float p_term = config_.kp * error;
    const float d_term = config_.kd * derivative;
    const float tentative_integral = integral_ + error * dt;

    // First pass: would integrating this step's error saturate the output further in the
    // direction the error is already pushing? If so, don't commit the tentative integral (see
    // the anti-windup note in pid.h) — recompute with the held-back integral instead.
    float unsaturated = p_term + config_.ki * tentative_integral + d_term;
    float saturated = Clamp(unsaturated, config_.output_min, config_.output_max);

    const bool saturating_high = unsaturated > saturated;
    const bool saturating_low = unsaturated < saturated;
    const bool integrating_further_into_saturation =
        (saturating_high && error > 0.0f) || (saturating_low && error < 0.0f);

    if (integrating_further_into_saturation) {
        unsaturated = p_term + config_.ki * integral_ + d_term;
        saturated = Clamp(unsaturated, config_.output_min, config_.output_max);
    } else {
        integral_ = tentative_integral;
    }

    previous_measurement_ = measurement;
    previous_error_ = error;
    have_previous_ = true;
    last_unsaturated_output_ = unsaturated;

    return saturated;
}

} // namespace bicopter
