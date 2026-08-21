#include "sim_loop.h"

#include <algorithm>
#include <cmath>

namespace bicopter {

namespace {
// Must match bicopter_dynamics.cpp's / complementary_filter.h's internal gravity constant (see
// docs/dynamics.md, docs/estimation.md).
constexpr float kGravityMps2 = 9.80665f;

// Body-frame specific force -- what an accelerometer actually measures (thrust + drag, excluding
// gravity's own contribution) -- derived from the world-frame acceleration Milestone 9's forward
// model computes for the given state/command. A stationary, level, unpowered vehicle has
// acceleration_mps2 == (0,0,g) (free fall in NED), so specific_force_world == 0, matching a
// free-falling accelerometer's true zero reading; a stationary, level, hovering vehicle has
// acceleration_mps2 == 0, so specific_force_world == (0,0,-g), which docs/estimation.md's
// "gravity is +Z is down" convention says is exactly what a level, at-rest accelerometer reads.
Vec3 computeSpecificForceBody(const RigidBodyState& state, const VehicleParams& params,
                               const ActuatorCommand& command)
{
    StateDerivative deriv = computeStateDerivative(state, params, command);
    Vec3 specific_force_world = deriv.acceleration_mps2 - Vec3(0.0f, 0.0f, kGravityMps2);
    return state.orientation.inverse().rotate(specific_force_world);
}
} // namespace

float quaternionAngleRad(const Quaternion& a, const Quaternion& b)
{
    Quaternion error = a.inverse() * b;
    float w = std::fabs(error.w);
    w = std::min(1.0f, std::max(-1.0f, w));
    return 2.0f * std::acos(w);
}

SimLoop::SimLoop(const SimLoopConfig& config)
    : config_(config),
      estimator_(config.estimator_config),
      attitude_controller_(config.attitude_controller_config),
      rate_controller_(config.rate_controller_config),
      allocator_(config.vehicle_params, config.allocator_config),
      imu_(config.imu_config)
{
    control_decimation_ = static_cast<unsigned>(
        std::max(1.0f, std::round(config_.control_period_s / config_.physics_dt_s)));
    reset(RigidBodyState{});
}

void SimLoop::reset(const RigidBodyState& initial_state, const Quaternion& initial_attitude_estimate)
{
    true_state_ = initial_state;
    estimator_.reset(initial_attitude_estimate);
    last_estimate_ = estimator_.estimate();
    rate_controller_.reset();
    imu_.reset();
    t_ = 0.0f;
    ticks_since_control_ = 0;

    desired_thrust_n_ = config_.desired_thrust_n > 0.0f
                             ? config_.desired_thrust_n
                             : config_.vehicle_params.mass_kg * kGravityMps2;

    // Hold hover trim (desired thrust, zero torque) until the first real control cycle runs.
    AllocatedCommand hover = allocator_.allocate(desired_thrust_n_, Vec3::Zero());
    last_command_.motor1_throttle = hover.motor1_throttle;
    last_command_.motor2_throttle = hover.motor2_throttle;
    last_command_.motor1_tilt_rad = hover.motor1_tilt_rad;
    last_command_.motor2_tilt_rad = hover.motor2_tilt_rad;

    last_log_ = SimStepLog{};
    last_log_.t_s = 0.0f;
    last_log_.true_orientation = true_state_.orientation;
    last_log_.estimated_orientation = last_estimate_.orientation;
    last_log_.true_attitude_error_rad = quaternionAngleRad(true_state_.orientation, desired_attitude_);
    last_log_.estimated_attitude_error_rad =
        quaternionAngleRad(last_estimate_.orientation, desired_attitude_);
    last_log_.command = hover;
}

void SimLoop::runControlCycle()
{
    last_estimate_ = estimator_.estimate();

    Vec3 desired_rate = attitude_controller_.update(desired_attitude_, last_estimate_.orientation);
    Vec3 desired_torque = rate_controller_.update(desired_rate, last_estimate_.angular_velocity_radps,
                                                    config_.control_period_s);
    AllocatedCommand allocated = allocator_.allocate(desired_thrust_n_, desired_torque);

    last_command_.motor1_throttle = allocated.motor1_throttle;
    last_command_.motor2_throttle = allocated.motor2_throttle;
    last_command_.motor1_tilt_rad = allocated.motor1_tilt_rad;
    last_command_.motor2_tilt_rad = allocated.motor2_tilt_rad;

    last_log_.t_s = t_;
    last_log_.true_orientation = true_state_.orientation;
    last_log_.estimated_orientation = last_estimate_.orientation;
    last_log_.true_attitude_error_rad = quaternionAngleRad(true_state_.orientation, desired_attitude_);
    last_log_.estimated_attitude_error_rad =
        quaternionAngleRad(last_estimate_.orientation, desired_attitude_);
    last_log_.command = allocated;
}

const SimStepLog& SimLoop::step()
{
    Vec3 specific_force_body = computeSpecificForceBody(true_state_, config_.vehicle_params, last_command_);
    auto sample = imu_.update(t_, true_state_, specific_force_body);
    if (sample.has_value()) {
        estimator_.update(*sample);
        ++ticks_since_control_;
        if (ticks_since_control_ >= control_decimation_) {
            ticks_since_control_ = 0;
            runControlCycle();
        }
    }

    true_state_ = stepRigidBodyState(true_state_, config_.vehicle_params, last_command_, config_.physics_dt_s);
    t_ += config_.physics_dt_s;

    return last_log_;
}

void SimLoop::run(float duration_s, const std::function<void(const SimStepLog&)>& trace)
{
    float last_logged_t = -1.0f;
    int steps = static_cast<int>(std::round(duration_s / config_.physics_dt_s));
    for (int i = 0; i < steps; ++i) {
        step();
        if (trace && last_log_.t_s != last_logged_t) {
            trace(last_log_);
            last_logged_t = last_log_.t_s;
        }
    }
}

} // namespace bicopter
