// Physical parameters of one specific built (or hypothesized) bicopter vehicle: mass, inertia,
// motor/servo geometry, and thrust/torque/drag coefficients. Pure configuration data, zero
// ESP-IDF/OS dependency, consistent with flight_core's "depends only on the hardware-abstraction
// interfaces" rule (AGENTS.md) — a config struct has no dependency at all.
//
// This lives in flight_core/vehicle/ (not simulator/physics/) per docs/hardware.md's "Open / TBD
// items" table and "Configuration philosophy" section, both written in Milestone 1: vehicle
// constants are meant to be consumed by both simulator/physics/'s forward-dynamics model
// (Milestone 9) and flight_core/control/'s allocation math (Milestone 12) from the same struct,
// so the two never drift apart. See docs/dynamics.md for the full derivation of every field's
// role in the physics model and the assumptions behind each one.
#pragma once

#include "vec3.h"

namespace bicopter {

// One motor + its tilt servo. A VehicleParams holds exactly two (motor1, motor2) per this
// project's fixed 2-motor/2-servo bicopter topology (docs/hardware.md's "Vehicle layout").
struct MotorParams {
    // This motor's position along body +Y (right), relative to the geometric reference frame's
    // origin — the symmetric point on the roll axis (body X) equidistant from both motors before
    // any center-of-mass offset is applied. E.g. -0.15 for the left motor, +0.15 for the right
    // motor. See docs/dynamics.md's "Geometry" section.
    float arm_offset_y_m = 0.0f;

    // Sign of this motor's reaction torque about its own (tilted) thrust axis: +1.0 or -1.0,
    // i.e. which way the prop spins. Motors need not spin opposite directions — see
    // docs/dynamics.md's "Spin direction" section for why this project doesn't assume that.
    float spin_direction = 1.0f;

    // Tilt-servo authority limits for this motor, radians, in this motor's own tilt frame (a
    // rotation about body +Y at zero vehicle attitude — see docs/dynamics.md's "Tilt-vectoring
    // geometry"). Placeholder defaults of +-30 deg per docs/hardware.md's "tens of degrees"
    // guidance; override with the actual built vehicle's mechanical limits.
    float min_tilt_rad = -0.5235988f;
    float max_tilt_rad = 0.5235988f;

    // thrust_n = thrust_coefficient_n * throttle^2, throttle in [0, 1] (MotorOutput's normalized
    // command convention, Milestone 5). See docs/dynamics.md's "Thrust model" section.
    float thrust_coefficient_n = 0.0f;

    // reaction_torque_nm = torque_coefficient_nm_per_n * thrust_n. See docs/dynamics.md's
    // "Reaction torque model" section.
    float torque_coefficient_nm_per_n = 0.0f;
};

struct VehicleParams {
    float mass_kg = 1.0f;

    // Diagonal (principal-axis) moment-of-inertia tensor, kg*m^2 — (Ixx, Iyy, Izz). See
    // docs/dynamics.md's "Inertia" section for why products of inertia are assumed negligible
    // for this milestone rather than carrying a full Mat3 tensor.
    Vec3 inertia_diag_kg_m2 = Vec3(0.01f, 0.01f, 0.02f);

    // True center of mass, offset from the geometric reference frame's origin (body frame,
    // meters) — see MotorParams::arm_offset_y_m and docs/dynamics.md's "Geometry" section.
    Vec3 center_of_mass_offset_m = Vec3::Zero();

    MotorParams motor1; // nominally the -Y (left) motor
    MotorParams motor2; // nominally the +Y (right) motor

    // Simple linear drag model, see docs/dynamics.md's "Drag" section — this is an explicitly
    // documented simplification, not real aerodynamics.
    float linear_drag_coefficient_n_per_mps = 0.0f;       // N per (m/s) of world-frame velocity
    float angular_drag_coefficient_nm_per_radps = 0.0f;   // N*m per (rad/s) of body angular rate
};

} // namespace bicopter
