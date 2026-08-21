// Rate (angular-velocity) controller: the first real control-loop consumer of the generic Pid
// building block (pid.h). Three independent PID loops — one each for roll, pitch, yaw body-rate
// error — with zero cross-coupling between axes inside this controller. Any cross-axis coupling a
// real bicopter exhibits is a property of the vehicle's rigid-body dynamics
// (simulator/physics/'s I*omega_dot + omega x (I*omega) = tau), not something this controller
// compensates for internally — see the axis-independence test in tests/rate_controller_test.cpp.
//
// Scope (Milestone 10): rate control only. This type consumes a desired body rate and the
// estimator's current body rate (flight_core/estimation/'s
// AttitudeEstimate::angular_velocity_radps) and produces a desired body torque. It does not
// compute rate setpoints from attitude error (Milestone 11's job) and does not convert torque
// into motor/servo commands (Milestone 12's control allocation consumes this type's output). See
// docs/control.md.
#pragma once

#include "pid.h"
#include "vec3.h"

namespace bicopter {

namespace detail {
// Placeholder rate-loop gains and torque saturation limits — not tuned against any real vehicle
// or simulator closed loop (Milestone 13's job) or real hardware (Milestone 17's). See
// docs/control.md for why these are deliberately labeled placeholders rather than presented as
// final tuned values; gains remain a config-time input (PidConfig/RateControllerConfig fields),
// never a hardcoded constant inside Pid's or RateController's logic.
inline PidConfig DefaultRateAxisGains()
{
    PidConfig config;
    config.kp = 0.08f;
    config.ki = 0.02f;
    config.kd = 0.004f;
    config.output_min = -0.5f; // N*m, placeholder — see docs/control.md
    config.output_max = 0.5f;  // N*m, placeholder — see docs/control.md
    config.derivative_on_measurement = true;
    return config;
}
} // namespace detail

struct RateControllerConfig {
    // One PID config per body axis. See detail::DefaultRateAxisGains() above for why the
    // defaults are explicitly placeholder values.
    PidConfig roll = detail::DefaultRateAxisGains();
    PidConfig pitch = detail::DefaultRateAxisGains();
    PidConfig yaw = detail::DefaultRateAxisGains();
};

class RateController {
public:
    explicit RateController(const RateControllerConfig& config = {});

    // Clears all three axes' integral/derivative history — call when (re)arming, or after a large
    // gap in control-loop execution. Same semantics as Pid::reset().
    void reset();

    // desired_radps / current_radps: body-frame angular velocity, rad/s, x=roll rate, y=pitch
    // rate, z=yaw rate, per README.md's body-frame convention (X=forward, Y=right, Z=down). dt:
    // seconds since the previous update() call — see Pid::update()'s dt handling, same semantics,
    // never assumed fixed.
    //
    // Returns the desired body torque, N*m, x=roll, y=pitch, z=yaw torque.
    Vec3 update(const Vec3& desired_radps, const Vec3& current_radps, float dt);

    const RateControllerConfig& config() const { return config_; }

private:
    RateControllerConfig config_;

    Pid roll_pid_;
    Pid pitch_pid_;
    Pid yaw_pid_;
};

} // namespace bicopter
