// Real automated tests for flight_core/math/mat3.{h,cpp}: identity/zero construction,
// matrix-vector multiply (the I*omega use case), matrix-matrix multiply, and transpose. Same
// hand-rolled assert-and-report harness as tests/bmp581_convert_test.c.

#include <cmath>
#include <cstdio>

#include "mat3.h"
#include "vec3.h"

using bicopter::Mat3;
using bicopter::Vec3;

static int g_failures = 0;
static int g_checks = 0;

#define CHECK(cond, msg)                                                                         \
    do {                                                                                          \
        g_checks++;                                                                               \
        if (!(cond)) {                                                                            \
            g_failures++;                                                                         \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, msg);                             \
        }                                                                                          \
    } while (0)

#define CHECK_NEAR(a, b, tol, msg) CHECK(std::fabs((a) - (b)) <= (tol), msg)

static void test_default_and_identity()
{
    const Mat3 zero;
    const Mat3 explicit_zero = Mat3::Zero();
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            CHECK_NEAR(zero(r, c), 0.0f, 1e-6f, "default-constructed Mat3 is all zero");
            CHECK_NEAR(explicit_zero(r, c), 0.0f, 1e-6f, "Mat3::Zero() entries are all zero");
        }
    }
    const Mat3 id = Mat3::Identity();
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            const float expected = (r == c) ? 1.0f : 0.0f;
            CHECK_NEAR(id(r, c), expected, 1e-6f, "identity matrix entries");
        }
    }
}

static void test_matrix_vector_multiply()
{
    const Mat3 id = Mat3::Identity();
    const Vec3 v(1.0f, 2.0f, 3.0f);
    CHECK((id * v) == v, "identity * v == v");

    // I*omega for a diagonal inertia tensor (the estimator/control use case this milestone
    // exists to support): each axis scales independently.
    const Mat3 inertia = Mat3::FromDiagonal(2.0f, 3.0f, 4.0f);
    const Vec3 omega(1.0f, 1.0f, 1.0f);
    const Vec3 h = inertia * omega;
    CHECK_NEAR(h.x, 2.0f, 1e-6f, "I*omega x for diagonal inertia tensor");
    CHECK_NEAR(h.y, 3.0f, 1e-6f, "I*omega y for diagonal inertia tensor");
    CHECK_NEAR(h.z, 4.0f, 1e-6f, "I*omega z for diagonal inertia tensor");

    const Mat3 rows = Mat3::FromRows(Vec3(1, 2, 3), Vec3(4, 5, 6), Vec3(7, 8, 9));
    const Vec3 result = rows * Vec3(1, 0, 0);
    CHECK(result == Vec3(1, 4, 7), "matrix-vector multiply picks out the first column");
}

static void test_matrix_matrix_multiply()
{
    const Mat3 id = Mat3::Identity();
    const Mat3 m = Mat3::FromRows(Vec3(1, 2, 3), Vec3(4, 5, 6), Vec3(7, 8, 9));

    const Mat3 m_times_id = m * id;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            CHECK_NEAR(m_times_id(r, c), m(r, c), 1e-6f, "m * identity == m");
        }
    }

    const Mat3 a = Mat3::FromRows(Vec3(1, 0, 0), Vec3(0, 1, 0), Vec3(0, 0, 1));
    const Mat3 b = Mat3::FromDiagonal(2.0f, 3.0f, 4.0f);
    const Mat3 product = a * b;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            CHECK_NEAR(product(r, c), b(r, c), 1e-6f, "identity * diagonal == diagonal");
        }
    }
}

static void test_transpose()
{
    const Mat3 m = Mat3::FromRows(Vec3(1, 2, 3), Vec3(4, 5, 6), Vec3(7, 8, 9));
    const Mat3 mt = m.transpose();
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            CHECK_NEAR(mt(r, c), m(c, r), 1e-6f, "transpose swaps indices");
        }
    }

    const Mat3 mtt = mt.transpose();
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            CHECK_NEAR(mtt(r, c), m(r, c), 1e-6f, "transpose of transpose == original");
        }
    }

    // A rotation matrix's transpose is its inverse: R * R^T == identity.
    // Build a simple rotation-like orthonormal matrix by hand (90 degrees about Z):
    // [0 -1 0; 1 0 0; 0 0 1]
    const Mat3 rot90z = Mat3::FromRows(Vec3(0, -1, 0), Vec3(1, 0, 0), Vec3(0, 0, 1));
    const Mat3 should_be_identity = rot90z * rot90z.transpose();
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            const float expected = (r == c) ? 1.0f : 0.0f;
            CHECK_NEAR(should_be_identity(r, c), expected, 1e-6f,
                       "R * R^T == identity for an orthonormal matrix");
        }
    }
}

static void test_trace()
{
    CHECK_NEAR(Mat3::Identity().trace(), 3.0f, 1e-6f, "trace of identity is 3");
    CHECK_NEAR(Mat3::FromDiagonal(1, 2, 3).trace(), 6.0f, 1e-6f, "trace sums diagonal entries");
}

int main()
{
    test_default_and_identity();
    test_matrix_vector_multiply();
    test_matrix_matrix_multiply();
    test_transpose();
    test_trace();

    std::printf("%d/%d checks passed\n", g_checks - g_failures, g_checks);
    return g_failures == 0 ? 0 : 1;
}
