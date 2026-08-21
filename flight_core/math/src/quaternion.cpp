#include "quaternion.h"

#include <cmath>

namespace bicopter {

namespace {
// Below this rotation angle (radians), integrate() uses the small-angle Taylor approximation of
// sin/cos instead of dividing by |omega| to build the rotation axis (which is ill-conditioned as
// |omega| -> 0). Below this norm, normalize() treats the quaternion as unrecoverable and returns
// Identity() rather than dividing by ~0.
constexpr float kSmallAngle = 1e-6f;
constexpr float kMinNormalizableNorm = 1e-9f;
constexpr float kHalfPi = 1.57079632679489661923f;
// How close sin(pitch) must be to +-1 before toEulerZYX() treats the attitude as gimbal-locked.
// Below this margin, the naive atan2 formulas for roll/yaw divide two quantities that both go to
// zero at the singularity, so their ratio is dominated by float rounding noise rather than the
// true (indeterminate) value.
constexpr float kGimbalLockSinPitchThreshold = 1.0f - 1e-6f;
} // namespace

Quaternion Quaternion::Identity()
{
    return Quaternion(1.0f, 0.0f, 0.0f, 0.0f);
}

Quaternion Quaternion::FromEulerZYX(const EulerAnglesZYX& euler)
{
    const float cr = std::cos(euler.roll * 0.5f);
    const float sr = std::sin(euler.roll * 0.5f);
    const float cp = std::cos(euler.pitch * 0.5f);
    const float sp = std::sin(euler.pitch * 0.5f);
    const float cy = std::cos(euler.yaw * 0.5f);
    const float sy = std::sin(euler.yaw * 0.5f);

    return Quaternion(cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
                       cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy);
}

Quaternion Quaternion::FromRotationMatrix(const Mat3& r)
{
    // Shepperd's method: pick the numerically best-conditioned branch (largest of the trace and
    // the three diagonal entries) rather than always dividing by a possibly-near-zero term.
    const float r00 = r(0, 0), r01 = r(0, 1), r02 = r(0, 2);
    const float r10 = r(1, 0), r11 = r(1, 1), r12 = r(1, 2);
    const float r20 = r(2, 0), r21 = r(2, 1), r22 = r(2, 2);
    const float trace = r00 + r11 + r22;

    Quaternion q;
    if (trace > 0.0f) {
        const float s = std::sqrt(trace + 1.0f) * 2.0f; // s = 4w
        q.w = 0.25f * s;
        q.x = (r21 - r12) / s;
        q.y = (r02 - r20) / s;
        q.z = (r10 - r01) / s;
    } else if (r00 > r11 && r00 > r22) {
        const float s = std::sqrt(1.0f + r00 - r11 - r22) * 2.0f; // s = 4x
        q.w = (r21 - r12) / s;
        q.x = 0.25f * s;
        q.y = (r01 + r10) / s;
        q.z = (r02 + r20) / s;
    } else if (r11 > r22) {
        const float s = std::sqrt(1.0f + r11 - r00 - r22) * 2.0f; // s = 4y
        q.w = (r02 - r20) / s;
        q.x = (r01 + r10) / s;
        q.y = 0.25f * s;
        q.z = (r12 + r21) / s;
    } else {
        const float s = std::sqrt(1.0f + r22 - r00 - r11) * 2.0f; // s = 4z
        q.w = (r10 - r01) / s;
        q.x = (r02 + r20) / s;
        q.y = (r12 + r21) / s;
        q.z = 0.25f * s;
    }
    return q.normalized();
}

Quaternion Quaternion::operator*(const Quaternion& rhs) const
{
    return Quaternion(w * rhs.w - x * rhs.x - y * rhs.y - z * rhs.z,
                       w * rhs.x + x * rhs.w + y * rhs.z - z * rhs.y,
                       w * rhs.y - x * rhs.z + y * rhs.w + z * rhs.x,
                       w * rhs.z + x * rhs.y - y * rhs.x + z * rhs.w);
}

Quaternion Quaternion::conjugate() const
{
    return Quaternion(w, -x, -y, -z);
}

Quaternion Quaternion::inverse() const
{
    const float n2 = normSquared();
    if (n2 < kMinNormalizableNorm) {
        return Quaternion::Identity();
    }
    const float inv = 1.0f / n2;
    return Quaternion(w * inv, -x * inv, -y * inv, -z * inv);
}

float Quaternion::normSquared() const
{
    return w * w + x * x + y * y + z * z;
}

float Quaternion::norm() const
{
    return std::sqrt(normSquared());
}

Quaternion Quaternion::normalized() const
{
    const float n = norm();
    if (n < kMinNormalizableNorm) {
        return Quaternion::Identity();
    }
    const float inv = 1.0f / n;
    return Quaternion(w * inv, x * inv, y * inv, z * inv);
}

void Quaternion::normalize()
{
    *this = normalized();
}

Quaternion Quaternion::integrate(const Vec3& omega, float dt) const
{
    const float theta = omega.length() * dt;
    Quaternion dq;
    if (theta < kSmallAngle) {
        // sin(theta/2)/theta -> 0.5, cos(theta/2) -> 1 as theta -> 0; this avoids normalizing a
        // near-zero axis vector while staying first-order accurate for tiny rotations.
        const float half_dt = 0.5f * dt;
        dq = Quaternion(1.0f, omega.x * half_dt, omega.y * half_dt, omega.z * half_dt);
        dq.normalize();
    } else {
        const Vec3 axis = omega.normalized();
        const float half_theta = theta * 0.5f;
        const float s = std::sin(half_theta);
        dq = Quaternion(std::cos(half_theta), axis.x * s, axis.y * s, axis.z * s);
    }
    return (*this * dq).normalized();
}

EulerAnglesZYX Quaternion::toEulerZYX() const
{
    EulerAnglesZYX out;

    float sin_pitch = 2.0f * (w * y - z * x);
    if (sin_pitch > 1.0f) {
        sin_pitch = 1.0f;
    } else if (sin_pitch < -1.0f) {
        sin_pitch = -1.0f;
    }

    // At pitch = +-90 deg the ZYX sequence loses a degree of freedom: only roll-yaw (at +90) or
    // roll+yaw (at -90) is determined by the rotation, not roll and yaw individually. The plain
    // atan2 formulas below are a 0/0 there (both their numerator and denominator vanish), so
    // instead take the documented convention roll = 0 and fold the whole remaining rotation into
    // yaw, computed directly from w/x (see docs/math.md for the derivation).
    if (sin_pitch > kGimbalLockSinPitchThreshold) {
        out.pitch = kHalfPi;
        out.roll = 0.0f;
        out.yaw = -2.0f * std::atan2(x, w);
    } else if (sin_pitch < -kGimbalLockSinPitchThreshold) {
        out.pitch = -kHalfPi;
        out.roll = 0.0f;
        out.yaw = 2.0f * std::atan2(x, w);
    } else {
        out.roll = std::atan2(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y));
        out.pitch = std::asin(sin_pitch);
        out.yaw = std::atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
    }

    return out;
}

Mat3 Quaternion::toRotationMatrix() const
{
    return Mat3::FromRows(
        Vec3(1.0f - 2.0f * (y * y + z * z), 2.0f * (x * y - w * z), 2.0f * (x * z + w * y)),
        Vec3(2.0f * (x * y + w * z), 1.0f - 2.0f * (x * x + z * z), 2.0f * (y * z - w * x)),
        Vec3(2.0f * (x * z - w * y), 2.0f * (y * z + w * x), 1.0f - 2.0f * (x * x + y * y)));
}

Vec3 Quaternion::rotate(const Vec3& v) const
{
    const Vec3 q_vec(x, y, z);
    const Vec3 t = cross(q_vec, v) * 2.0f;
    return v + t * w + cross(q_vec, t);
}

} // namespace bicopter
