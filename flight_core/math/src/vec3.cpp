#include "vec3.h"

#include <cmath>

namespace bicopter {

namespace {
// Below this length, normalize() treats the vector as zero rather than amplifying float noise
// into an arbitrary unit direction.
constexpr float kMinNormalizableLength = 1e-9f;
} // namespace

Vec3 Vec3::Zero()
{
    return Vec3(0.0f, 0.0f, 0.0f);
}

Vec3 Vec3::operator+(const Vec3& rhs) const
{
    return Vec3(x + rhs.x, y + rhs.y, z + rhs.z);
}

Vec3 Vec3::operator-(const Vec3& rhs) const
{
    return Vec3(x - rhs.x, y - rhs.y, z - rhs.z);
}

Vec3 Vec3::operator-() const
{
    return Vec3(-x, -y, -z);
}

Vec3 Vec3::operator*(float s) const
{
    return Vec3(x * s, y * s, z * s);
}

Vec3 Vec3::operator/(float s) const
{
    return Vec3(x / s, y / s, z / s);
}

Vec3& Vec3::operator+=(const Vec3& rhs)
{
    x += rhs.x;
    y += rhs.y;
    z += rhs.z;
    return *this;
}

Vec3& Vec3::operator-=(const Vec3& rhs)
{
    x -= rhs.x;
    y -= rhs.y;
    z -= rhs.z;
    return *this;
}

Vec3& Vec3::operator*=(float s)
{
    x *= s;
    y *= s;
    z *= s;
    return *this;
}

Vec3& Vec3::operator/=(float s)
{
    x /= s;
    y /= s;
    z /= s;
    return *this;
}

bool Vec3::operator==(const Vec3& rhs) const
{
    return x == rhs.x && y == rhs.y && z == rhs.z;
}

bool Vec3::operator!=(const Vec3& rhs) const
{
    return !(*this == rhs);
}

float Vec3::lengthSquared() const
{
    return x * x + y * y + z * z;
}

float Vec3::length() const
{
    return std::sqrt(lengthSquared());
}

Vec3 Vec3::normalized() const
{
    const float len = length();
    if (len < kMinNormalizableLength) {
        return Vec3::Zero();
    }
    return *this * (1.0f / len);
}

void Vec3::normalize()
{
    *this = normalized();
}

Vec3 operator*(float s, const Vec3& v)
{
    return v * s;
}

float dot(const Vec3& a, const Vec3& b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3 cross(const Vec3& a, const Vec3& b)
{
    return Vec3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

} // namespace bicopter
