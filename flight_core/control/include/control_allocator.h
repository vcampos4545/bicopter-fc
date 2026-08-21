// Control allocation: the last stage of the cascaded control architecture
// (docs/architecture.md), converting a desired total thrust and body torque (Milestone 10-11's
// RateController output, plus a desired-thrust scalar from whatever future altitude/throttle
// source supplies one) into the vehicle's 4 actual actuator degrees of freedom: motor1 throttle,
// motor2 throttle, motor1 tilt, motor2 tilt.
//
// This is the mathematical inverse of Milestone 9's forward-dynamics model
// (simulator/physics/bicopter_dynamics.cpp's computeStateDerivative()/computeMotorEffect()), and
// per this project's design brief, that inverse must be *derived* from the same geometry, not
// invented. See docs/control_allocation.md for:
//   - The full derivation, term by term, from Milestone 9's r x F / reaction-torque equations to
//     the small-angle-linearized closed-form solved here.
//   - The single most load-bearing finding of this milestone: with this vehicle's geometry (both
//     motors on the body Y axis, tilt axis also body Y) and VehicleParams::center_of_mass_offset_m
//     at its default Vec3::Zero(), this configuration has NO pitch-torque authority near hover —
//     not a limitation of this allocator, a structural fact about the vehicle geometry as
//     currently parameterized. Real pitch authority requires a nonzero vertical
//     (center_of_mass_offset_m.z) offset between the motor-mount plane and the true center of
//     mass; the allocator below uses that offset exactly if configured nonzero, and degrades to a
//     documented, non-crashing fallback if it's zero (the default).
//   - The saturation/prioritization policy applied when a desired thrust+torque combination isn't
//     achievable within throttle/tilt limits.
//
// Reuses VehicleParams (flight_core/vehicle/) directly, and flight_core/vehicle/motor_geometry.h's
// motorThrustDirectionBody() for the exact same thrust-direction convention Milestone 9 verified
// in tests/bicopter_dynamics_test.cpp — see that header's comment for why the *function itself*
// (not just its formula) can be shared across this dependency edge but not the reverse.
#pragma once

#include "vec3.h"
#include "vehicle_params.h"

namespace bicopter {

struct ControlAllocatorConfig {
    // Normalized throttle floor/ceiling applied to both motors, [0,1] — the flight_core-side
    // analogue of Milestone 5's pwm_esc_convert_config_t::min_throttle/max_throttle (that struct
    // is firmware-only/ESP-IDF-adjacent and can't be depended on from flight_core; this is the
    // same concept re-expressed as a plain config flight_core owns). Defaults to the full [0,1]
    // range Milestone 5's MotorOutput::write() accepts. Values are clamped to [0,1] and swapped if
    // configured inverted, mirroring pwm_esc_convert.h's defensive-config handling.
    float throttle_min = 0.0f;
    float throttle_max = 1.0f;
};

// The 4 actuator commands, in the exact normalized units Milestone 5's MotorOutput
// (throttle, [0,1]) / ServoOutput (angle, radians) interfaces expect. Deliberately a distinct
// type from simulator/physics/'s ActuatorCommand (same field shape, same units) rather than the
// same type, because flight_core/control/ cannot depend on simulator/physics/ (see this header's
// top comment) — tests that round-trip this allocator's output through Milestone 9's forward
// model construct a simulator::ActuatorCommand from these fields explicitly at the call site.
struct AllocatedCommand {
    float motor1_throttle = 0.0f;
    float motor2_throttle = 0.0f;
    float motor1_tilt_rad = 0.0f;
    float motor2_tilt_rad = 0.0f;

    // True if any requested thrust/torque component had to be clamped or dropped to stay within
    // configured throttle/tilt limits (or a structurally-unachievable-axis fallback was used —
    // see docs/control_allocation.md's "Saturation and prioritization" section). A caller/
    // telemetry consumer can use this to detect "the vehicle is being asked for more authority
    // than it has," distinct from a normal in-envelope command.
    bool saturated = false;
};

// Stateless — a pure function of (VehicleParams, ControlAllocatorConfig, desired thrust/torque),
// same "no integral/derivative history to reset()" style as AttitudeController (Milestone 11).
class ControlAllocator {
public:
    explicit ControlAllocator(const VehicleParams& params,
                               const ControlAllocatorConfig& config = {});

    // desired_thrust_n: total desired thrust magnitude along body -Z (Newtons, e.g. from a future
    // altitude/throttle source; motor1_thrust_n + motor2_thrust_n in the ideal, unsaturated
    // solution). desired_torque_nm: desired body torque, N*m, x=roll/y=pitch/z=yaw — the direct
    // output shape of RateController::update() (Milestone 10).
    //
    // Returns the 4 actuator commands that best achieve that thrust/torque within this
    // ControlAllocator's configured throttle/tilt limits. See docs/control_allocation.md for the
    // full derivation and the documented saturation/prioritization policy; never returns NaN/Inf
    // or an out-of-limit command, even for a wildly unachievable request.
    AllocatedCommand allocate(float desired_thrust_n, const Vec3& desired_torque_nm) const;

    const VehicleParams& vehicleParams() const { return params_; }
    const ControlAllocatorConfig& config() const { return config_; }

private:
    VehicleParams params_;
    ControlAllocatorConfig config_;
};

} // namespace bicopter
