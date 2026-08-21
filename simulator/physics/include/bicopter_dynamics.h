// Rigid-body forward-dynamics model for the bicopter: given a vehicle's current full state and
// an actuator command, computes the resulting motion. This is the Milestone 9 deliverable — see
// docs/dynamics.md for the full derivation of every equation and documented assumption below.
//
// Scope: forward dynamics only (state + command -> motion). Control allocation (desired
// thrust/torque -> actuator command, the inverse problem) is flight_core/control/'s Milestone 12
// job, not this file's.
#pragma once

#include "actuator_command.h"
#include "rigid_body_state.h"
#include "vehicle_params.h"

namespace bicopter {

// Instantaneous rate of change of a RigidBodyState — the right-hand side of m*v_dot=F and
// I*omega_dot + omega x (I*omega) = tau. Exposed separately from stepRigidBodyState() so tests
// can inspect instantaneous forces/torques directly (e.g. "does hover thrust produce zero net
// acceleration") without integrating over a timestep first.
struct StateDerivative {
    Vec3 velocity_mps = Vec3::Zero();                  // = d(position)/dt, world frame
    Vec3 acceleration_mps2 = Vec3::Zero();             // = d(velocity)/dt, world frame
    Vec3 angular_acceleration_radps2 = Vec3::Zero();   // = d(angular_velocity)/dt, body frame
};

// Direction (unit vector) of one motor's thrust vector in BODY frame at the given tilt angle: a
// rotation of tilt_rad about body +Y (right-hand rule) applied to the zero-tilt thrust axis
// (0, 0, -1) — straight "up" in this project's Z-down body frame. See docs/dynamics.md's
// "Tilt-vectoring geometry" section for the derivation and sign convention.
Vec3 motorThrustDirectionBody(float tilt_rad);

// Computes the state derivative for the given state, vehicle parameters, and actuator command,
// per m*v_dot=F and I*omega_dot + omega x (I*omega) = tau (solved for omega_dot assuming a
// diagonal inertia tensor — see docs/dynamics.md's "Inertia" and "Rotational dynamics"
// sections). A pure function: no internal state, safe to call repeatedly without side effects.
StateDerivative computeStateDerivative(const RigidBodyState& state, const VehicleParams& params,
                                        const ActuatorCommand& command);

// Advances state forward by dt (seconds) using semi-implicit (symplectic) Euler integration for
// translation and angular velocity, and Quaternion::integrate() (Milestone 7's exact
// exponential-map body-rate integrator — not hand-rolled quaternion math) for orientation. See
// docs/dynamics.md's "Integration scheme" section for why semi-implicit Euler was chosen.
RigidBodyState stepRigidBodyState(const RigidBodyState& state, const VehicleParams& params,
                                   const ActuatorCommand& command, float dt);

} // namespace bicopter
