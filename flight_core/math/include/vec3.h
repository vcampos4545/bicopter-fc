// 3D vector: single-precision (matches this project's ESP32 target's single-precision FPU and
// the SI-units/body-frame convention fixed in README.md and AGENTS.md). Zero ESP-IDF/OS
// dependency — see docs/math.md for the full flight_core/math/ convention writeup.
#pragma once

namespace bicopter {

struct Vec3 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    Vec3() = default;
    Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    static Vec3 Zero();

    Vec3 operator+(const Vec3& rhs) const;
    Vec3 operator-(const Vec3& rhs) const;
    Vec3 operator-() const;
    Vec3 operator*(float s) const;
    Vec3 operator/(float s) const;

    Vec3& operator+=(const Vec3& rhs);
    Vec3& operator-=(const Vec3& rhs);
    Vec3& operator*=(float s);
    Vec3& operator/=(float s);

    bool operator==(const Vec3& rhs) const;
    bool operator!=(const Vec3& rhs) const;

    float lengthSquared() const;
    float length() const;

    // Returns Zero() for a (near-)zero-length vector rather than dividing by ~0.
    Vec3 normalized() const;
    void normalize();
};

Vec3 operator*(float s, const Vec3& v);

float dot(const Vec3& a, const Vec3& b);
Vec3 cross(const Vec3& a, const Vec3& b);

} // namespace bicopter
