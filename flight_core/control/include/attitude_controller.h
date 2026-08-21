// Attitude (angle) controller: the outer loop of the cascaded attitude/rate architecture
// described in docs/architecture.md (attitude error -> rate setpoint -> Milestone 10's
// RateController -> body torque). Platform-independent, no ESP-IDF dependency (per AGENTS.md's
// flight_core rule).
//
// Scope (Milestone 11): outer loop only. This type consumes a desired and current attitude
// quaternion and produces a desired body rate — it feeds RateController::update()'s
// desired_radps argument unchanged. It does not touch RateController's interface and does not
// compute torque, thrust, or motor/servo commands (Milestone 12's control allocation).
//
// ## Quaternion error convention
//
// Both `desired` and `current` are body-to-world (NED) attitude quaternions, per
// docs/math.md's convention. The error is computed as
//
//     q_error = current.inverse() * desired
//
// i.e. current-frame-first, NOT `desired.inverse() * current`. This direction is deliberate, not
// arbitrary — it's the one that stays consistent with Quaternion::integrate()'s documented
// right-multiply convention (q(t+dt) = normalize(q(t) * dq(omega_body, dt)), where dq is a small
// rotation expressed in the current body frame's own axes, i.e. exactly what a gyro/body-rate
// command means). Solving that same relationship for the rotation that would carry `current` to
// `desired` in one step, dq === q_error such that current * q_error = desired, gives
// q_error = current.inverse() * desired directly. This is important to get right: the opposite
// order (`desired.inverse() * current`) produces the vector part with the opposite sign, which
// silently flips the control direction (commands the vehicle to rotate away from the setpoint
// instead of toward it) while still compiling and looking plausible in isolation — see
// tests/attitude_controller_test.cpp's single-axis-direction tests, which pin this down
// numerically, and docs/control.md for the full derivation with a worked example.
//
// ## Small-angle proportional law
//
// For a rotation of angle theta about unit axis n, a quaternion's vector part is
// sin(theta/2)*n ~= (theta/2)*n for small theta. So q_error.{x,y,z} approximates
// (angle_error/2) * axis, and multiplying by 2 recovers an "angle-times-axis" rotation vector
// whose magnitude approximates the angular error (radians) and whose direction is the body-frame
// axis to rotate about to reduce it — exactly the shape of a desired body rate. Per-axis
// proportional gains are applied directly to that rotation vector's components (config_.kp),
// matching the axis-independent style RateController already established at Milestone 10 rather
// than gain on a combined vector norm. This is the standard "quaternion feedback" technique for
// rigid-body attitude control (see e.g. Wie, Weiss & Arapostathis 1989; the small-angle P-only
// form used here is the simplified building block, without the full nonlinear/Lyapunov
// derivation those references also cover).
//
// **Small-angle assumption and its limit:** the theta/2 ~= sin(theta/2) approximation degrades as
// theta grows — by 90 degrees it under-commands by about 10%, and the whole linear picture breaks
// down as theta approaches 180 degrees (where the vector part saturates at magnitude 1 instead of
// growing with theta, so error direction stays correct but magnitude is heavily under-read). This
// is not compensated for. See docs/control.md for the quantitative error curve and why it's
// judged acceptable for this vehicle's flight envelope.
//
// **180-degree handling:** before computing the vector part, if q_error.w < 0 this
// implementation negates all four of q_error's components. This is NOT special 180-degree
// handling in the sense of a dedicated flip-recovery maneuver — it is the standard fix for
// quaternion double-cover (q and -q represent the identical physical rotation, but the "short
// way around" vs. "long way around" representations of an equivalent error have opposite-sign
// vector parts). Forcing w >= 0 selects the representation whose rotation angle is in [0, 180]
// degrees, so the controller always commands the shorter of the two directions. It does NOT
// resolve the genuine singularity at theta == exactly 180 degrees (axis choice is ambiguous
// there — either direction is equally short), and it does not implement any dedicated
// upside-down recovery logic. See docs/control.md for why this is judged fine for a bicopter
// (attitude stabilization near level flight, not an aerobatic vehicle expected to recover from a
// full inversion).
#pragma once

#include "quaternion.h"
#include "vec3.h"

namespace bicopter {

namespace detail {
// Placeholder attitude-loop gains and rate-command limits — not tuned against any real vehicle or
// simulator closed loop (Milestone 13's job) or real hardware (Milestone 17's). Same "identical
// value on every axis, deliberately not physically motivated" placeholder approach
// RateController's detail::DefaultRateAxisGains() used at Milestone 10 — see docs/control.md.
inline Vec3 DefaultAttitudeKp()
{
    return Vec3(4.0f, 4.0f, 4.0f); // (rad/s commanded rate) per (rad of attitude error), placeholder
}

inline Vec3 DefaultRateLimitRadps()
{
    return Vec3(6.0f, 6.0f, 6.0f); // rad/s, placeholder soft cap on the outer loop's rate demand
}
} // namespace detail

struct AttitudeControllerConfig {
    // Per-axis proportional gain, x=roll, y=pitch, z=yaw. Units: (rad/s of commanded body rate)
    // per (rad of small-angle attitude error) — see the file header's derivation of the *2
    // factor. No gain is a hardcoded constant inside AttitudeController's logic; every caller
    // supplies its own config, same convention as PidConfig/RateControllerConfig.
    Vec3 kp = detail::DefaultAttitudeKp();

    // Symmetric per-axis clamp on the commanded body rate, rad/s, applied to the total
    // (proportional term + feedforward) before it's returned. Always applied (same "always-on
    // saturation" style as Pid's output_min/output_max) — this is what keeps a large attitude
    // error from demanding an unrealistically large rate of RateController. A non-positive value
    // on an axis disables the limit for that axis (treated as "not configured" rather than "clamp
    // to zero").
    Vec3 rate_limit_radps = detail::DefaultRateLimitRadps();
};

// Stateless by design: unlike Pid/RateController, this is a pure function of the current pair of
// quaternions (and an optional feedforward), so there is no integral/derivative history to
// reset() between calls. If a later milestone adds an integral or derivative term here, this
// class gains real state and a reset() then.
class AttitudeController {
public:
    explicit AttitudeController(const AttitudeControllerConfig& config = {});

    // desired / current: body-to-world (NED) attitude quaternions, per docs/math.md's convention
    // — current is meant to come directly from flight_core/estimation/'s
    // AttitudeEstimate::orientation (Milestone 8). feedforward_radps: an optional additive
    // desired body rate (e.g. a trajectory's own commanded rate), Vec3::Zero() by default when
    // there is no feedforward source — see the file header and docs/control.md for why this is
    // optional and not yet exercised by any real caller.
    //
    // Returns the desired body rate, rad/s, x=roll y=pitch z=yaw — feeds RateController::update()
    // desired_radps argument unchanged.
    Vec3 update(const Quaternion& desired, const Quaternion& current,
                const Vec3& feedforward_radps = Vec3::Zero()) const;

    const AttitudeControllerConfig& config() const { return config_; }
    void setConfig(const AttitudeControllerConfig& config) { config_ = config; }

private:
    AttitudeControllerConfig config_;
};

} // namespace bicopter
