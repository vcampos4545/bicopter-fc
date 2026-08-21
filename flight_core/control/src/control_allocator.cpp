// See control_allocator.h and docs/control_allocation.md for the full derivation this
// implementation follows step by step. Short version of the notation used throughout (matches
// docs/control_allocation.md exactly so the two can be read side by side):
//
//   y_i   = params.motor_i.arm_offset_y_m
//   c     = params.center_of_mass_offset_m
//   d_i   = y_i - c.y                              (Y moment arm relative to the true CoM)
//   k_i   = params.motor_i.thrust_coefficient_n
//   s_i   = params.motor_i.spin_direction
//   q_i   = params.motor_i.torque_coefficient_nm_per_n
//
// Small-angle linearization around hover (tilt_i small): thrust_dir_i ~= (-tilt_i, 0, -1), so
// each motor's body-frame force is F_i ~= thrust_i * (-tilt_i, 0, -1). Substituting into
// r_i x F_i plus the reaction-torque term (both from bicopter_dynamics.cpp's computeMotorEffect())
// and dropping second-order-in-tilt terms gives:
//
//   T     = thrust_1 + thrust_2                                          (total thrust)
//   tau_x = -d_1*thrust_1 - d_2*thrust_2                                 (roll)
//   tau_y = c.z*(thrust_1*tilt_1 + thrust_2*tilt_2) - c.x*T              (pitch)
//   tau_z = d_1*thrust_1*tilt_1 + d_2*thrust_2*tilt_2
//           - (s_1*q_1*thrust_1 + s_2*q_2*thrust_2)                     (yaw)
//
// which decouples into two independent 2x2 linear solves: (T, tau_x) -> (thrust_1, thrust_2),
// then (tau_y, tau_z) -> (tilt_1, tilt_2) given those now-known thrust values.
#include "control_allocator.h"

#include <cmath>

namespace bicopter {

namespace {

// Numerical-robustness thresholds only -- these are not physical mixing coefficients (nothing
// here influences *how* the vehicle is commanded to move; they only decide when a linear solve's
// denominator is too close to zero to trust, i.e. when to use the documented fallback below
// instead of dividing by ~0). VehicleParams' geometric fields default to exactly zero
// (Vec3::Zero()), so an exact-zero check would already catch the common case; a small epsilon
// additionally covers a configured value that's nonzero but numerically negligible.
constexpr float kGeometryEpsilon = 1e-6f;
constexpr float kThrustEpsilon = 1e-6f; // Newtons, below which a motor is treated as "off"

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

struct ThrustBounds {
    float min_n = 0.0f;
    float max_n = 0.0f;
};

ThrustBounds motorThrustBounds(const MotorParams& motor, float throttle_min, float throttle_max)
{
    ThrustBounds bounds;
    if (motor.thrust_coefficient_n <= 0.0f) {
        return bounds; // motor can't produce thrust regardless of throttle -- [0,0]
    }
    bounds.min_n = motor.thrust_coefficient_n * throttle_min * throttle_min;
    bounds.max_n = motor.thrust_coefficient_n * throttle_max * throttle_max;
    return bounds;
}

} // namespace

ControlAllocator::ControlAllocator(const VehicleParams& params,
                                    const ControlAllocatorConfig& config)
    : params_(params), config_(config)
{
    // Defensive clamp/swap, same style as pwm_esc_convert.h's config handling: a malformed config
    // can't invert or escape the [0,1] normalized-throttle range.
    config_.throttle_min = clamp(config_.throttle_min, 0.0f, 1.0f);
    config_.throttle_max = clamp(config_.throttle_max, 0.0f, 1.0f);
    if (config_.throttle_min > config_.throttle_max) {
        const float tmp = config_.throttle_min;
        config_.throttle_min = config_.throttle_max;
        config_.throttle_max = tmp;
    }
}

AllocatedCommand ControlAllocator::allocate(float desired_thrust_n,
                                             const Vec3& desired_torque_nm) const
{
    AllocatedCommand out;

    const ThrustBounds bounds1 =
        motorThrustBounds(params_.motor1, config_.throttle_min, config_.throttle_max);
    const ThrustBounds bounds2 =
        motorThrustBounds(params_.motor2, config_.throttle_min, config_.throttle_max);

    // --- Stage 0: clamp total thrust to what the two motors can jointly deliver -----------------
    // Thrust is prioritized over torque accuracy throughout this function (documented in
    // docs/control_allocation.md's "Saturation and prioritization" section): losing lift is worse
    // than losing some roll/pitch/yaw authority, so every later stage treats the thrust solved
    // here as fixed and lets torque error absorb the remaining saturation.
    const float total_min_n = bounds1.min_n + bounds2.min_n;
    const float total_max_n = bounds1.max_n + bounds2.max_n;
    float thrust_desired = desired_thrust_n;
    if (thrust_desired < total_min_n || thrust_desired > total_max_n) {
        out.saturated = true;
    }
    const float total_thrust_n = clamp(thrust_desired, total_min_n, total_max_n);

    // --- Stage 1: (total_thrust_n, tau_x) -> (thrust_1, thrust_2) -------------------------------
    // thrust_1 + thrust_2 = T
    // -d_1*thrust_1 - d_2*thrust_2 = tau_x
    // Determinant of that 2x2 system is (d_2 - d_1) = (y_2 - y_1) (the center_of_mass_offset_m.y
    // term cancels) -- see docs/control_allocation.md for the algebra.
    const float d1 = params_.motor1.arm_offset_y_m - params_.center_of_mass_offset_m.y;
    const float d2 = params_.motor2.arm_offset_y_m - params_.center_of_mass_offset_m.y;
    const float roll_det = d2 - d1;

    float thrust1_ideal;
    float thrust2_ideal;
    if (std::fabs(roll_det) < kGeometryEpsilon) {
        // Degenerate geometry: both motors at (numerically) the same Y position, so differential
        // thrust cannot produce roll torque at all. Falls back to an even split -- roll authority
        // is genuinely unavailable here, not something a different split could recover; see
        // docs/control_allocation.md.
        thrust1_ideal = total_thrust_n * 0.5f;
        thrust2_ideal = total_thrust_n * 0.5f;
        if (std::fabs(desired_torque_nm.x) > kGeometryEpsilon) {
            out.saturated = true;
        }
    } else {
        thrust1_ideal = (desired_torque_nm.x + d2 * total_thrust_n) / roll_det;
        thrust2_ideal = total_thrust_n - thrust1_ideal;
    }

    // Clamp thrust_1 to its own limits first, then re-derive thrust_2 to preserve total thrust
    // exactly if possible (thrust priority over roll accuracy -- see Stage 0's comment). Only if
    // thrust_2 *also* saturates does total thrust itself end up compromised, which only happens
    // when the requested (thrust, roll) combination is outside the vehicle's combined envelope.
    float thrust1 = clamp(thrust1_ideal, bounds1.min_n, bounds1.max_n);
    if (thrust1 != thrust1_ideal) {
        out.saturated = true;
    }
    float thrust2 = total_thrust_n - thrust1;
    const float thrust2_preclamp = thrust2;
    thrust2 = clamp(thrust2, bounds2.min_n, bounds2.max_n);
    if (thrust2 != thrust2_preclamp) {
        out.saturated = true;
    }

    // --- Stage 2: (tau_y, tau_z) -> (tilt_1, tilt_2), given the now-fixed thrust_1/thrust_2 ------
    // Substituting u_i = thrust_i * tilt_i (linear in tilt_i since thrust_i is now a constant):
    //   c.z*(u_1 + u_2)        = tau_y + c.x*T_actual
    //   d_1*u_1 + d_2*u_2      = tau_z + (s_1*q_1*thrust_1 + s_2*q_2*thrust_2)
    // Determinant is c.z*(d_2-d_1) -- zero whenever center_of_mass_offset_m.z is zero (the
    // VehicleParams default), which is exactly the "this geometry has no pitch-torque authority
    // near hover" finding documented in docs/control_allocation.md. When that happens, pitch is
    // honestly reported as unachievable (not silently dropped) and yaw alone is solved via pure
    // differential tilt (u_1 = -u_2).
    const float com_z = params_.center_of_mass_offset_m.z;
    const float com_x = params_.center_of_mass_offset_m.x;
    const float total_thrust_actual = thrust1 + thrust2;
    const float reaction_yaw_bias = params_.motor1.spin_direction *
                                         params_.motor1.torque_coefficient_nm_per_n * thrust1 +
                                     params_.motor2.spin_direction *
                                         params_.motor2.torque_coefficient_nm_per_n * thrust2;

    const float pitch_rhs = desired_torque_nm.y + com_x * total_thrust_actual;
    const float yaw_rhs = desired_torque_nm.z + reaction_yaw_bias;
    const float pitch_det = com_z * (d2 - d1);

    float u1;
    float u2;
    if (std::fabs(pitch_det) < kGeometryEpsilon) {
        // Pitch is structurally unachievable via tilt at this center_of_mass_offset_m -- see the
        // header comment above and docs/control_allocation.md. Report it rather than pretending
        // it was honored.
        if (std::fabs(pitch_rhs) > kGeometryEpsilon) {
            out.saturated = true;
        }
        if (std::fabs(roll_det) < kGeometryEpsilon) {
            // Doubly-degenerate geometry (both motors coincident in Y as well): yaw's own
            // fallback below would divide by ~0 too. No lever left for either axis -- hold tilt
            // at zero rather than producing NaN/garbage.
            u1 = 0.0f;
            u2 = 0.0f;
            if (std::fabs(yaw_rhs) > kGeometryEpsilon) {
                out.saturated = true;
            }
        } else {
            // Pure differential tilt (u_1 = -u_2) solves yaw alone using whatever authority
            // remains once pitch is given up on.
            u1 = yaw_rhs / (d1 - d2);
            u2 = -u1;
        }
    } else {
        // Cramer's rule for [[com_z, com_z], [d1, d2]] * [u1, u2] = [pitch_rhs, yaw_rhs].
        u1 = (pitch_rhs * d2 - com_z * yaw_rhs) / pitch_det;
        u2 = (com_z * yaw_rhs - d1 * pitch_rhs) / pitch_det;
    }

    float tilt1_ideal = (thrust1 > kThrustEpsilon) ? (u1 / thrust1) : 0.0f;
    float tilt2_ideal = (thrust2 > kThrustEpsilon) ? (u2 / thrust2) : 0.0f;

    float tilt1 = clamp(tilt1_ideal, params_.motor1.min_tilt_rad, params_.motor1.max_tilt_rad);
    if (tilt1 != tilt1_ideal) {
        out.saturated = true;
    }
    float tilt2 = clamp(tilt2_ideal, params_.motor2.min_tilt_rad, params_.motor2.max_tilt_rad);
    if (tilt2 != tilt2_ideal) {
        out.saturated = true;
    }

    // --- Throttle conversion: thrust_i = k_i * throttle_i^2, inverted -------------------------
    out.motor1_throttle = (params_.motor1.thrust_coefficient_n > 0.0f)
                               ? std::sqrt(thrust1 / params_.motor1.thrust_coefficient_n)
                               : 0.0f;
    out.motor2_throttle = (params_.motor2.thrust_coefficient_n > 0.0f)
                               ? std::sqrt(thrust2 / params_.motor2.thrust_coefficient_n)
                               : 0.0f;
    out.motor1_tilt_rad = tilt1;
    out.motor2_tilt_rad = tilt2;

    return out;
}

} // namespace bicopter
