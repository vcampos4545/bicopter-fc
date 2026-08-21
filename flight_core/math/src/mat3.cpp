#include "mat3.h"

namespace bicopter {

Mat3::Mat3()
{
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            m_[r][c] = 0.0f;
        }
    }
}

Mat3 Mat3::Zero()
{
    return Mat3();
}

Mat3 Mat3::Identity()
{
    Mat3 out;
    out.m_[0][0] = 1.0f;
    out.m_[1][1] = 1.0f;
    out.m_[2][2] = 1.0f;
    return out;
}

Mat3 Mat3::FromRows(const Vec3& row0, const Vec3& row1, const Vec3& row2)
{
    Mat3 out;
    out.m_[0][0] = row0.x;
    out.m_[0][1] = row0.y;
    out.m_[0][2] = row0.z;
    out.m_[1][0] = row1.x;
    out.m_[1][1] = row1.y;
    out.m_[1][2] = row1.z;
    out.m_[2][0] = row2.x;
    out.m_[2][1] = row2.y;
    out.m_[2][2] = row2.z;
    return out;
}

Mat3 Mat3::FromDiagonal(float d0, float d1, float d2)
{
    Mat3 out;
    out.m_[0][0] = d0;
    out.m_[1][1] = d1;
    out.m_[2][2] = d2;
    return out;
}

float& Mat3::operator()(int row, int col)
{
    return m_[row][col];
}

float Mat3::operator()(int row, int col) const
{
    return m_[row][col];
}

Mat3 Mat3::operator+(const Mat3& rhs) const
{
    Mat3 out;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            out.m_[r][c] = m_[r][c] + rhs.m_[r][c];
        }
    }
    return out;
}

Mat3 Mat3::operator-(const Mat3& rhs) const
{
    Mat3 out;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            out.m_[r][c] = m_[r][c] - rhs.m_[r][c];
        }
    }
    return out;
}

Mat3 Mat3::operator*(float s) const
{
    Mat3 out;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            out.m_[r][c] = m_[r][c] * s;
        }
    }
    return out;
}

Mat3 Mat3::operator*(const Mat3& rhs) const
{
    Mat3 out;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            float sum = 0.0f;
            for (int k = 0; k < 3; ++k) {
                sum += m_[r][k] * rhs.m_[k][c];
            }
            out.m_[r][c] = sum;
        }
    }
    return out;
}

Vec3 Mat3::operator*(const Vec3& v) const
{
    return Vec3(m_[0][0] * v.x + m_[0][1] * v.y + m_[0][2] * v.z,
                m_[1][0] * v.x + m_[1][1] * v.y + m_[1][2] * v.z,
                m_[2][0] * v.x + m_[2][1] * v.y + m_[2][2] * v.z);
}

Mat3 Mat3::transpose() const
{
    Mat3 out;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            out.m_[c][r] = m_[r][c];
        }
    }
    return out;
}

float Mat3::trace() const
{
    return m_[0][0] + m_[1][1] + m_[2][2];
}

} // namespace bicopter
