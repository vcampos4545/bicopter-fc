// Real automated tests for flight_core/math/quaternion.{h,cpp}: identity, Hamilton-product
// multiplication, conjugate/inverse, normalization, angular-velocity integration, and
// Euler-angle/rotation-matrix conversion round trips (including the gimbal-lock edge case). Same
// hand-rolled assert-and-report harness as tests/bmp581_convert_test.c.
//
// Convention under test (see flight_core/math/include/quaternion.h and docs/math.md): Hamilton
// product, scalar-first [w,x,y,z], q rotates a body-frame vector into world frame, ZYX
// (yaw-pitch-roll) Euler sequence matching README.md's roll-X/pitch-Y/yaw-Z body axes.

#include <cmath>
#include <cstdio>

#include "quaternion.h"
#include "vec3.h"

using bicopter::EulerAnglesZYX;
using bicopter::Quaternion;
using bicopter::Vec3;

namespace {
constexpr float kPi = 3.14159265358979323846f;
} // namespace

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

// Quaternions q and -q represent the same rotation; compare allowing for that sign ambiguity.
static bool quat_near(const Quaternion& a, const Quaternion& b, float tol)
{
    const bool same_sign = std::fabs(a.w - b.w) <= tol && std::fabs(a.x - b.x) <= tol &&
                            std::fabs(a.y - b.y) <= tol && std::fabs(a.z - b.z) <= tol;
    const bool flipped_sign = std::fabs(a.w + b.w) <= tol && std::fabs(a.x + b.x) <= tol &&
                               std::fabs(a.y + b.y) <= tol && std::fabs(a.z + b.z) <= tol;
    return same_sign || flipped_sign;
}

static bool vec_near(const Vec3& a, const Vec3& b, float tol)
{
    return std::fabs(a.x - b.x) <= tol && std::fabs(a.y - b.y) <= tol &&
           std::fabs(a.z - b.z) <= tol;
}

static void test_identity()
{
    const Quaternion id = Quaternion::Identity();
    CHECK_NEAR(id.w, 1.0f, 1e-6f, "identity w");
    CHECK_NEAR(id.x, 0.0f, 1e-6f, "identity x");
    CHECK_NEAR(id.y, 0.0f, 1e-6f, "identity y");
    CHECK_NEAR(id.z, 0.0f, 1e-6f, "identity z");

    const Vec3 v(1.0f, 2.0f, 3.0f);
    CHECK(vec_near(id.rotate(v), v, 1e-6f), "identity quaternion rotate() is a no-op");
    CHECK(vec_near(id.toRotationMatrix() * v, v, 1e-6f), "identity rotation matrix is a no-op");
}

static void test_multiplication()
{
    // 90-degree rotations about the principal axes: q(axis, theta) = [cos(theta/2),
    // axis*sin(theta/2)].
    const float half = kPi / 4.0f; // 90 deg / 2
    const Quaternion qx(std::cos(half), std::sin(half), 0.0f, 0.0f);
    const Quaternion qy(std::cos(half), 0.0f, std::sin(half), 0.0f);
    const Quaternion qz(std::cos(half), 0.0f, 0.0f, std::sin(half));

    // Composing a 90-deg X rotation with itself twice should give a 180-deg X rotation.
    const Quaternion qx_twice = qx * qx;
    CHECK(quat_near(qx_twice, Quaternion(0.0f, 1.0f, 0.0f, 0.0f), 1e-5f),
          "two 90-deg X rotations compose to a 180-deg X rotation");

    // Multiplying by identity on either side is a no-op.
    CHECK(quat_near(qx * Quaternion::Identity(), qx, 1e-6f), "q * identity == q");
    CHECK(quat_near(Quaternion::Identity() * qx, qx, 1e-6f), "identity * q == q");

    // A world-frame X vector rotated 90 deg about Z should land on world-frame Y (right-handed
    // basis, matches this project's right-handed body/world frames).
    const Vec3 rotated = qz.rotate(Vec3(1, 0, 0));
    CHECK(vec_near(rotated, Vec3(0, 1, 0), 1e-5f), "90-deg Z rotation maps X onto Y");

    (void)qy; // exercised indirectly via toEulerZYX/rotation-matrix tests below
}

static void test_conjugate_and_inverse()
{
    const Quaternion q(0.7071068f, 0.7071068f, 0.0f, 0.0f); // 90-deg X rotation, already unit

    const Quaternion conj = q.conjugate();
    CHECK_NEAR(conj.w, q.w, 1e-6f, "conjugate preserves scalar part");
    CHECK_NEAR(conj.x, -q.x, 1e-6f, "conjugate negates x");
    CHECK_NEAR(conj.y, -q.y, 1e-6f, "conjugate negates y");
    CHECK_NEAR(conj.z, -q.z, 1e-6f, "conjugate negates z");

    const Quaternion should_be_identity = q * q.inverse();
    CHECK(quat_near(should_be_identity, Quaternion::Identity(), 1e-5f),
          "q * q.inverse() == identity");

    // For a unit quaternion, inverse() and conjugate() coincide.
    CHECK(quat_near(q.inverse(), q.conjugate(), 1e-6f),
          "inverse() == conjugate() for a unit quaternion");

    // For a non-unit quaternion, inverse() still recovers identity under multiplication.
    const Quaternion scaled(1.4142136f, 1.4142136f, 0.0f, 0.0f); // q * 2
    const Quaternion scaled_inv_check = scaled * scaled.inverse();
    CHECK(quat_near(scaled_inv_check, Quaternion::Identity(), 1e-4f),
          "non-unit q * q.inverse() == identity");
}

static void test_normalization()
{
    const Quaternion unnormalized(2.0f, 0.0f, 0.0f, 0.0f);
    const Quaternion n = unnormalized.normalized();
    CHECK_NEAR(n.norm(), 1.0f, 1e-6f, "normalized() yields unit norm");
    CHECK_NEAR(n.w, 1.0f, 1e-6f, "normalizing (2,0,0,0) yields identity");

    Quaternion already_unit = Quaternion::Identity();
    already_unit.normalize();
    CHECK(quat_near(already_unit, Quaternion::Identity(), 1e-6f),
          "normalizing an already-unit quaternion is a no-op");

    // Degenerate case: normalizing a (near-)zero quaternion must not divide by zero / produce
    // NaN; falls back to Identity() per the documented contract.
    const Quaternion zero(0.0f, 0.0f, 0.0f, 0.0f);
    CHECK(quat_near(zero.normalized(), Quaternion::Identity(), 1e-6f),
          "normalizing a zero quaternion yields identity, not NaN");
}

static void test_integrate_constant_angular_velocity()
{
    // Property test: integrating a constant angular rate omega for time T should produce the
    // same rotation as directly constructing the axis-angle quaternion for angle = |omega|*T.
    const Quaternion start = Quaternion::Identity();
    const Vec3 omega(0.0f, 0.0f, kPi); // pi rad/s about Z
    const float dt = 1.0f;             // 1 second -> pi radians == 180 deg about Z

    // Integrate in many small steps, as EstimatorTask would each sensor cycle, to also exercise
    // that repeated small-step integration doesn't drift from the single-shot closed form.
    const int steps = 1000;
    Quaternion q = start;
    for (int i = 0; i < steps; ++i) {
        q = q.integrate(omega, dt / steps);
    }

    const Quaternion expected(0.0f, 0.0f, 0.0f, 1.0f); // 180 deg about Z: [cos(pi/2), 0,0,sin(pi/2)]
    CHECK(quat_near(q, expected, 1e-4f),
          "integrating a constant angular velocity for a known time produces the expected "
          "rotation");
    CHECK_NEAR(q.norm(), 1.0f, 1e-5f, "repeated integration steps stay unit norm");

    // A single large step covering the same total rotation should match too (the exponential-map
    // integrator is exact for constant angular velocity, regardless of step size).
    const Quaternion single_step = start.integrate(omega, dt);
    CHECK(quat_near(single_step, expected, 1e-5f),
          "single-step integration over the full interval matches the many-small-steps result");

    // Integrating with dt = 0 (or omega = 0) must be a no-op, not a division-by-zero / NaN.
    const Quaternion no_motion = start.integrate(Vec3::Zero(), 0.01f);
    CHECK(quat_near(no_motion, Quaternion::Identity(), 1e-6f),
          "integrating zero angular velocity is a no-op");
    const Quaternion no_time = start.integrate(omega, 0.0f);
    CHECK(quat_near(no_time, Quaternion::Identity(), 1e-6f), "integrating over zero dt is a no-op");
}

static void test_euler_round_trip()
{
    // Representative attitudes well away from the pitch = +-90 deg gimbal-lock singularity.
    const float kDeg = kPi / 180.0f;
    const EulerAnglesZYX cases[] = {
        {0.0f, 0.0f, 0.0f},
        {30.0f * kDeg, 0.0f, 0.0f},
        {0.0f, 20.0f * kDeg, 0.0f},
        {0.0f, 0.0f, 45.0f * kDeg},
        {15.0f * kDeg, -25.0f * kDeg, 60.0f * kDeg},
        {-45.0f * kDeg, 60.0f * kDeg, -120.0f * kDeg},
        {170.0f * kDeg, 10.0f * kDeg, -170.0f * kDeg},
    };

    for (const EulerAnglesZYX& e : cases) {
        const Quaternion q = Quaternion::FromEulerZYX(e);
        const EulerAnglesZYX round_tripped = q.toEulerZYX();

        CHECK_NEAR(round_tripped.roll, e.roll, 1e-3f, "Euler round trip: roll");
        CHECK_NEAR(round_tripped.pitch, e.pitch, 1e-3f, "Euler round trip: pitch");
        CHECK_NEAR(round_tripped.yaw, e.yaw, 1e-3f, "Euler round trip: yaw");

        // Also round-trip through the quaternion representation itself (not just the angles),
        // since angle-only comparison could mask a q vs -q sign flip.
        const Quaternion rebuilt = Quaternion::FromEulerZYX(round_tripped);
        CHECK(quat_near(q, rebuilt, 1e-3f), "quaternion round trip via Euler angles");
    }
}

static void test_euler_gimbal_lock()
{
    // At pitch = +90 deg, roll and yaw become degenerate (only roll+yaw, or roll-yaw, is
    // determined). Documented behavior: toEulerZYX() reports roll == 0 and folds the combined
    // rotation into yaw. This test locks in that documented behavior rather than asserting a
    // clean round trip, which is mathematically impossible at the singularity.
    const EulerAnglesZYX gimbal_lock_input{0.3f, kPi / 2.0f, 0.5f};
    const Quaternion q = Quaternion::FromEulerZYX(gimbal_lock_input);
    const EulerAnglesZYX out = q.toEulerZYX();

    CHECK_NEAR(out.pitch, kPi / 2.0f, 1e-3f, "gimbal lock: pitch still recovered as +90 deg");
    CHECK_NEAR(out.roll, 0.0f, 1e-3f, "gimbal lock: roll collapses to 0 by convention");

    // The quaternion built from the degenerate (roll=0, pitch=90, combined-yaw) output must
    // still represent the SAME physical rotation as the original input, even though the
    // individual roll/yaw split differs.
    const Quaternion rebuilt = Quaternion::FromEulerZYX(out);
    CHECK(quat_near(q, rebuilt, 1e-3f),
          "gimbal lock: recovered angles reconstruct the same physical rotation");

    // Same check at pitch = -90 deg.
    const EulerAnglesZYX neg_gimbal_lock_input{0.3f, -kPi / 2.0f, 0.5f};
    const Quaternion q_neg = Quaternion::FromEulerZYX(neg_gimbal_lock_input);
    const EulerAnglesZYX out_neg = q_neg.toEulerZYX();
    CHECK_NEAR(out_neg.pitch, -kPi / 2.0f, 1e-3f, "gimbal lock at -90 deg: pitch recovered");
    CHECK_NEAR(out_neg.roll, 0.0f, 1e-3f, "gimbal lock at -90 deg: roll collapses to 0");
}

static void test_rotation_matrix_round_trip()
{
    const float kDeg = kPi / 180.0f;
    const EulerAnglesZYX cases[] = {
        {0.0f, 0.0f, 0.0f},
        {25.0f * kDeg, -40.0f * kDeg, 100.0f * kDeg},
        {-80.0f * kDeg, 30.0f * kDeg, -10.0f * kDeg},
    };

    for (const EulerAnglesZYX& e : cases) {
        const Quaternion q = Quaternion::FromEulerZYX(e);
        const auto r = q.toRotationMatrix();

        // Orthonormality: R * R^T == identity for any valid rotation matrix.
        const auto should_be_identity = r * r.transpose();
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                const float expected = (row == col) ? 1.0f : 0.0f;
                CHECK_NEAR(should_be_identity(row, col), expected, 1e-4f,
                           "rotation matrix is orthonormal (R * R^T == I)");
            }
        }

        // rotate() must agree with the equivalent matrix-vector multiply.
        const Vec3 v(1.0f, -2.0f, 0.5f);
        CHECK(vec_near(q.rotate(v), r * v, 1e-4f),
              "quaternion rotate() matches toRotationMatrix() * v");

        // Quaternion -> matrix -> quaternion round trip (up to sign).
        const Quaternion rebuilt = Quaternion::FromRotationMatrix(r);
        CHECK(quat_near(q, rebuilt, 1e-3f), "rotation matrix round trip recovers the quaternion");
    }
}

int main()
{
    test_identity();
    test_multiplication();
    test_conjugate_and_inverse();
    test_normalization();
    test_integrate_constant_angular_velocity();
    test_euler_round_trip();
    test_euler_gimbal_lock();
    test_rotation_matrix_round_trip();

    std::printf("%d/%d checks passed\n", g_checks - g_failures, g_checks);
    return g_failures == 0 ? 0 : 1;
}
