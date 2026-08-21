// Real automated tests for flight_core/math/vec3.{h,cpp}: arithmetic operators, dot/cross
// products, and normalization (including the degenerate zero-vector case). Same hand-rolled
// assert-and-report harness as tests/bmp581_convert_test.c (no external test framework available
// offline in this environment), exiting non-zero on any failure so it works as a real CI gate.

#include <cmath>
#include <cstdio>

#include "vec3.h"

using bicopter::cross;
using bicopter::dot;
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

static void test_arithmetic()
{
    const Vec3 a(1.0f, 2.0f, 3.0f);
    const Vec3 b(4.0f, -5.0f, 6.0f);

    const Vec3 sum = a + b;
    CHECK_NEAR(sum.x, 5.0f, 1e-6f, "add x");
    CHECK_NEAR(sum.y, -3.0f, 1e-6f, "add y");
    CHECK_NEAR(sum.z, 9.0f, 1e-6f, "add z");

    const Vec3 diff = a - b;
    CHECK_NEAR(diff.x, -3.0f, 1e-6f, "sub x");
    CHECK_NEAR(diff.y, 7.0f, 1e-6f, "sub y");
    CHECK_NEAR(diff.z, -3.0f, 1e-6f, "sub z");

    const Vec3 neg = -a;
    CHECK_NEAR(neg.x, -1.0f, 1e-6f, "unary negate x");

    const Vec3 scaled = a * 2.0f;
    CHECK_NEAR(scaled.x, 2.0f, 1e-6f, "scale x");
    CHECK_NEAR(scaled.z, 6.0f, 1e-6f, "scale z");

    const Vec3 scaled_lhs = 2.0f * a;
    CHECK(scaled_lhs == scaled, "scalar*vec matches vec*scalar");

    const Vec3 divided = a / 2.0f;
    CHECK_NEAR(divided.x, 0.5f, 1e-6f, "divide x");

    Vec3 accum = a;
    accum += b;
    CHECK(accum == sum, "+= matches +");

    accum = a;
    accum -= b;
    CHECK(accum == diff, "-= matches -");

    accum = a;
    accum *= 2.0f;
    CHECK(accum == scaled, "*= matches *");

    accum = a;
    accum /= 2.0f;
    CHECK(accum == divided, "/= matches /");

    CHECK(a != b, "distinct vectors compare unequal");
    CHECK(a == Vec3(1.0f, 2.0f, 3.0f), "equal vectors compare equal");
}

static void test_dot()
{
    CHECK_NEAR(dot(Vec3(1, 0, 0), Vec3(0, 1, 0)), 0.0f, 1e-6f, "orthogonal unit vectors dot to 0");
    CHECK_NEAR(dot(Vec3(1, 2, 3), Vec3(1, 2, 3)), 14.0f, 1e-6f, "self dot == length squared");
    CHECK_NEAR(dot(Vec3(1, 0, 0), Vec3(-1, 0, 0)), -1.0f, 1e-6f, "opposite unit vectors dot to -1");
}

static void test_cross()
{
    // Right-handed basis: X cross Y == Z (matches this project's right-handed body/world frames).
    const Vec3 x_cross_y = cross(Vec3(1, 0, 0), Vec3(0, 1, 0));
    CHECK(x_cross_y == Vec3(0, 0, 1), "x cross y == z");

    const Vec3 y_cross_z = cross(Vec3(0, 1, 0), Vec3(0, 0, 1));
    CHECK(y_cross_z == Vec3(1, 0, 0), "y cross z == x");

    const Vec3 z_cross_x = cross(Vec3(0, 0, 1), Vec3(1, 0, 0));
    CHECK(z_cross_x == Vec3(0, 1, 0), "z cross x == y");

    const Vec3 a(1.0f, 2.0f, 3.0f);
    const Vec3 b(4.0f, 5.0f, 6.0f);
    const Vec3 c = cross(a, b);
    CHECK_NEAR(dot(c, a), 0.0f, 1e-4f, "cross product orthogonal to first operand");
    CHECK_NEAR(dot(c, b), 0.0f, 1e-4f, "cross product orthogonal to second operand");

    CHECK(cross(a, a) == Vec3::Zero(), "cross of a vector with itself is zero");
}

static void test_length_and_normalize()
{
    CHECK_NEAR(Vec3(3, 4, 0).length(), 5.0f, 1e-6f, "3-4-5 triangle length");
    CHECK_NEAR(Vec3(3, 4, 0).lengthSquared(), 25.0f, 1e-6f, "length squared avoids the sqrt");

    const Vec3 n = Vec3(3, 4, 0).normalized();
    CHECK_NEAR(n.length(), 1.0f, 1e-6f, "normalized vector has unit length");
    CHECK_NEAR(n.x, 0.6f, 1e-6f, "normalized x component");
    CHECK_NEAR(n.y, 0.8f, 1e-6f, "normalized y component");

    // Degenerate case: normalizing the zero vector must not divide by zero / produce NaN.
    const Vec3 zero_normalized = Vec3::Zero().normalized();
    CHECK(zero_normalized == Vec3::Zero(), "normalizing the zero vector yields zero, not NaN");

    Vec3 in_place(6, 8, 0);
    in_place.normalize();
    CHECK_NEAR(in_place.length(), 1.0f, 1e-6f, "in-place normalize yields unit length");
    CHECK_NEAR(in_place.x, 0.6f, 1e-6f, "in-place normalize x component");
}

int main()
{
    test_arithmetic();
    test_dot();
    test_cross();
    test_length_and_normalize();

    std::printf("%d/%d checks passed\n", g_checks - g_failures, g_checks);
    return g_failures == 0 ? 0 : 1;
}
