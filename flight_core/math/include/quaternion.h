// Attitude quaternion. See docs/math.md for the full convention writeup; summary:
//
// - Hamilton product, right-handed, scalar-first storage order [w, x, y, z].
// - A Quaternion represents the rotation FROM body frame TO world frame (NED): for a vector
//   v_body expressed in body-frame coordinates, q.rotate(v_body) / q.toRotationMatrix() * v_body
//   gives the same physical vector expressed in world-frame coordinates.
// - Body-frame angular velocity (as read from a gyro) integrates on the RIGHT:
//   q(t+dt) = normalize(q(t) * dq(omega_body, dt)) — see integrate() below.
// - Euler sequence: ZYX / yaw-pitch-roll (intrinsic: yaw about world Z, then pitch about the new
//   Y, then roll about the new X), i.e. R = Rz(yaw) * Ry(pitch) * Rx(roll). This matches
//   README.md's body-frame axes (roll about X, pitch about Y, yaw about Z) and is the standard
//   aerospace 3-2-1 sequence.
#pragma once

#include "mat3.h"
#include "vec3.h"

namespace bicopter {

// Roll (about body X), pitch (about body Y), yaw (about body Z), radians, ZYX sequence.
struct EulerAnglesZYX {
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
};

class Quaternion {
public:
    float w = 1.0f;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    Quaternion() = default;
    Quaternion(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_) {}

    static Quaternion Identity();
    static Quaternion FromEulerZYX(const EulerAnglesZYX& euler);
    static Quaternion FromRotationMatrix(const Mat3& r);

    // Hamilton product: this * rhs (apply rhs first, then this, when both act on the same
    // reference frame's vectors; body-rate integration instead right-multiplies, see integrate).
    Quaternion operator*(const Quaternion& rhs) const;

    Quaternion conjugate() const;
    // Full inverse (conjugate / normSquared); equals conjugate() for a unit quaternion, but does
    // the right thing for a not-yet-normalized one too.
    Quaternion inverse() const;

    float normSquared() const;
    float norm() const;
    // Returns Identity() if the quaternion's norm is (near) zero rather than dividing by ~0.
    Quaternion normalized() const;
    void normalize();

    // Integrates body-frame angular velocity omega (rad/s) over dt (s) using the exponential-map
    // update q(t+dt) = normalize(q(t) * dq), dq = [cos(|omega|dt/2), axis*sin(|omega|dt/2)]. See
    // docs/math.md for why this was chosen over the first-order q_dot = 0.5*q*omega_quat step.
    Quaternion integrate(const Vec3& omega, float dt) const;

    // Degenerate (gimbal-lock) at pitch = +-90 deg: roll is taken to be 0 and yaw absorbs the
    // combined rotation. See docs/math.md and tests/quaternion_test.cpp for the exact behavior.
    EulerAnglesZYX toEulerZYX() const;
    Mat3 toRotationMatrix() const;

    // Rotates v (expressed in body frame) into world frame. Equivalent to
    // toRotationMatrix() * v but avoids building the intermediate matrix.
    Vec3 rotate(const Vec3& v) const;
};

} // namespace bicopter
