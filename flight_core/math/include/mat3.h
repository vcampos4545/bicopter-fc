// Fixed-size 3x3 matrix: just enough for rotation matrices and the inertia-tensor use case
// (I * omega) the estimator/control milestones need. Not a general NxN linear-algebra library —
// see docs/math.md for the scoping rationale. Row-major storage, zero ESP-IDF/OS dependency.
#pragma once

#include "vec3.h"

namespace bicopter {

class Mat3 {
public:
    // Default-constructs to the zero matrix (consistent with Vec3's zero default) — callers that
    // want a rotation/identity matrix must ask for Mat3::Identity() explicitly.
    Mat3();

    static Mat3 Zero();
    static Mat3 Identity();
    static Mat3 FromRows(const Vec3& row0, const Vec3& row1, const Vec3& row2);
    static Mat3 FromDiagonal(float d0, float d1, float d2);

    float& operator()(int row, int col);
    float operator()(int row, int col) const;

    Mat3 operator+(const Mat3& rhs) const;
    Mat3 operator-(const Mat3& rhs) const;
    Mat3 operator*(float s) const;
    Mat3 operator*(const Mat3& rhs) const;
    Vec3 operator*(const Vec3& v) const;

    Mat3 transpose() const;
    float trace() const;

private:
    float m_[3][3];
};

} // namespace bicopter
