// Full kinematic state of the simulated vehicle at one instant. World-frame position/velocity,
// body-to-world orientation, body-frame angular velocity — the same quantities an AttitudeEstimator
// (flight_core/estimation/) publishes, plus position/velocity the estimator doesn't (yet) cover.
// See docs/dynamics.md for the equations that advance this state forward in time.
#pragma once

#include "quaternion.h"
#include "vec3.h"

namespace bicopter {

struct RigidBodyState {
    Vec3 position_m = Vec3::Zero();                    // world frame (NED), meters
    Vec3 velocity_mps = Vec3::Zero();                   // world frame (NED), m/s
    Quaternion orientation = Quaternion::Identity();    // body-to-world, see docs/math.md
    Vec3 angular_velocity_radps = Vec3::Zero();         // body frame, rad/s
};

} // namespace bicopter
