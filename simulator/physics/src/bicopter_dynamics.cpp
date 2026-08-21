#include "bicopter_dynamics.h"

#include <cmath>

namespace bicopter {

namespace {

// Standard gravity, m/s^2. World frame is NED (+Z down), so the gravity force vector on the
// vehicle is (0, 0, +mass*kGravityMps2) — see docs/dynamics.md's "Forces" section.
constexpr float kGravityMps2 = 9.80665f;

float clamp(float value, float lo, float hi)
{
    if (value < lo) {
        return lo;
    }
    if (value > hi) {
        return hi;
    }
    return value;
}

struct MotorEffect {
    Vec3 force_body_n;   // thrust force vector, body frame, Newtons
    Vec3 torque_body_nm; // r x F plus reaction torque, body frame, N*m
};

// Force and torque (about the center of mass) one motor contributes, given its own parameters
// and commanded throttle/tilt. Throttle and tilt are clamped here (not trusted from the caller),
// mirroring firmware/components/actuators's convert-at-the-boundary pattern (AGENTS.md's driver-
// testing convention).
MotorEffect computeMotorEffect(const MotorParams& motor, const Vec3& center_of_mass_offset_m,
                                float throttle_cmd, float tilt_cmd_rad)
{
    const float throttle = clamp(throttle_cmd, 0.0f, 1.0f);
    const float tilt = clamp(tilt_cmd_rad, motor.min_tilt_rad, motor.max_tilt_rad);

    const float thrust_n = motor.thrust_coefficient_n * throttle * throttle;
    const Vec3 thrust_dir = motorThrustDirectionBody(tilt);
    const Vec3 force = thrust_dir * thrust_n;

    // Motor position relative to the center of mass: its fixed geometric offset along body +Y,
    // minus how far the true CoM sits from that geometric reference frame's origin.
    const Vec3 motor_position_ref(0.0f, motor.arm_offset_y_m, 0.0f);
    const Vec3 r = motor_position_ref - center_of_mass_offset_m;

    // Reaction torque acts about the motor's own (tilted) spin axis, i.e. the same direction as
    // its thrust vector — see docs/dynamics.md's "Reaction torque model" section.
    const Vec3 reaction_torque =
        thrust_dir * (motor.spin_direction * motor.torque_coefficient_nm_per_n * thrust_n);

    MotorEffect out;
    out.force_body_n = force;
    out.torque_body_nm = cross(r, force) + reaction_torque;
    return out;
}

} // namespace

Vec3 motorThrustDirectionBody(float tilt_rad)
{
    // Rotation of tilt_rad about body +Y (right-hand rule) applied to (0, 0, -1):
    //   x' =  0*cos(t) + (-1)*sin(t) = -sin(t)
    //   y' =  0
    //   z' = -0*sin(t) + (-1)*cos(t) = -cos(t)
    // At tilt=0 this is (0,0,-1) — straight up. Positive tilt rotates the thrust vector toward
    // -X (aft). See docs/dynamics.md's "Tilt-vectoring geometry" section.
    return Vec3(-std::sin(tilt_rad), 0.0f, -std::cos(tilt_rad));
}

StateDerivative computeStateDerivative(const RigidBodyState& state, const VehicleParams& params,
                                        const ActuatorCommand& command)
{
    const MotorEffect m1 = computeMotorEffect(params.motor1, params.center_of_mass_offset_m,
                                               command.motor1_throttle, command.motor1_tilt_rad);
    const MotorEffect m2 = computeMotorEffect(params.motor2, params.center_of_mass_offset_m,
                                               command.motor2_throttle, command.motor2_tilt_rad);

    // --- Translational dynamics: m*v_dot = F -------------------------------------------------
    const Vec3 gravity_force_world(0.0f, 0.0f, params.mass_kg * kGravityMps2);
    const Vec3 thrust_force_world = state.orientation.rotate(m1.force_body_n + m2.force_body_n);
    const Vec3 drag_force_world =
        state.velocity_mps * (-params.linear_drag_coefficient_n_per_mps);

    const Vec3 net_force_world = gravity_force_world + thrust_force_world + drag_force_world;

    StateDerivative out;
    out.velocity_mps = state.velocity_mps;
    out.acceleration_mps2 = net_force_world / params.mass_kg;

    // --- Rotational dynamics: I*omega_dot + omega x (I*omega) = tau --------------------------
    // I is assumed diagonal (see VehicleParams::inertia_diag_kg_m2 and docs/dynamics.md's
    // "Inertia" section), so I*omega and the final solve for omega_dot are both a per-axis
    // (elementwise) operation rather than a general 3x3 matrix solve/inverse.
    const Vec3& omega = state.angular_velocity_radps;
    const Vec3& i_diag = params.inertia_diag_kg_m2;
    const Vec3 i_omega(i_diag.x * omega.x, i_diag.y * omega.y, i_diag.z * omega.z);
    const Vec3 gyroscopic_torque = cross(omega, i_omega);
    const Vec3 drag_torque_body = omega * (-params.angular_drag_coefficient_nm_per_radps);

    const Vec3 net_torque_body =
        m1.torque_body_nm + m2.torque_body_nm + drag_torque_body - gyroscopic_torque;

    out.angular_acceleration_radps2 = Vec3(net_torque_body.x / i_diag.x,
                                            net_torque_body.y / i_diag.y,
                                            net_torque_body.z / i_diag.z);

    return out;
}

RigidBodyState stepRigidBodyState(const RigidBodyState& state, const VehicleParams& params,
                                   const ActuatorCommand& command, float dt)
{
    const StateDerivative deriv = computeStateDerivative(state, params, command);

    // Semi-implicit (symplectic) Euler: integrate velocity first, then use the UPDATED velocity/
    // angular velocity to advance position/orientation — see docs/dynamics.md's "Integration
    // scheme" section for why this was chosen over plain forward Euler.
    RigidBodyState next;
    next.velocity_mps = state.velocity_mps + deriv.acceleration_mps2 * dt;
    next.position_m = state.position_m + next.velocity_mps * dt;

    next.angular_velocity_radps =
        state.angular_velocity_radps + deriv.angular_acceleration_radps2 * dt;
    next.orientation = state.orientation.integrate(next.angular_velocity_radps, dt);

    return next;
}

} // namespace bicopter
