// Real automated tests for flight_core/control/attitude_controller.{h,cpp}: zero attitude error
// produces zero commanded rate, an isolated roll/pitch/yaw attitude error each produces a
// commanded rate in the correct direction and expected magnitude, a combined multi-axis error
// combines correctly per axis, the shortest-path (quaternion double-cover) fix picks the shorter
// direction for a large-angle error, and the rate limit actually clamps. Same hand-rolled
// assert-and-report harness as tests/pid_test.cpp / tests/rate_controller_test.cpp.
//
// Expected values are computed independently in each test from Quaternion::FromEulerZYX (a
// Milestone 7 primitive already tested on its own in tests/quaternion_test.cpp) plus the exact
// closed-form sin(theta/2) relationship documented in attitude_controller.h's file header, not by
// re-deriving attitude_controller.cpp's own arithmetic.

#include <cmath>
#include <cstdio>

#include "attitude_controller.h"
#include "quaternion.h"
#include "vec3.h"

using bicopter::AttitudeController;
using bicopter::AttitudeControllerConfig;
using bicopter::EulerAnglesZYX;
using bicopter::Quaternion;
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

static AttitudeControllerConfig UnlimitedConfig(float kp)
{
    AttitudeControllerConfig config;
    config.kp = Vec3(kp, kp, kp);
    // Non-positive == "no limit configured", per the config field's doc comment — lets these
    // tests inspect the raw proportional (+ feedforward) output without the rate limit
    // interfering, since saturation is covered by its own dedicated test below.
    config.rate_limit_radps = Vec3(0.0f, 0.0f, 0.0f);
    return config;
}

static void test_zero_error_produces_zero_rate()
{
    AttitudeController controller(UnlimitedConfig(4.0f));

    // Trivial case: both at identity.
    const Vec3 out_identity = controller.update(Quaternion::Identity(), Quaternion::Identity());
    CHECK_NEAR(out_identity.x, 0.0f, 1e-6f, "identity vs identity: zero roll rate");
    CHECK_NEAR(out_identity.y, 0.0f, 1e-6f, "identity vs identity: zero pitch rate");
    CHECK_NEAR(out_identity.z, 0.0f, 1e-6f, "identity vs identity: zero yaw rate");

    // Non-trivial case: desired == current at some arbitrary non-identity attitude. Confirms
    // zero-error-produces-zero-rate isn't just a trivial identity-vs-identity artifact.
    const Quaternion attitude = Quaternion::FromEulerZYX(EulerAnglesZYX{0.3f, -0.2f, 0.6f});
    const Vec3 out_matched = controller.update(attitude, attitude);
    CHECK_NEAR(out_matched.x, 0.0f, 1e-5f, "matched non-identity attitudes: zero roll rate");
    CHECK_NEAR(out_matched.y, 0.0f, 1e-5f, "matched non-identity attitudes: zero pitch rate");
    CHECK_NEAR(out_matched.z, 0.0f, 1e-5f, "matched non-identity attitudes: zero yaw rate");
}

static void test_single_axis_roll_error()
{
    const float kp = 4.0f;
    AttitudeController controller(UnlimitedConfig(kp));

    const float theta = 0.1f; // rad, small-angle regime
    const Quaternion desired = Quaternion::FromEulerZYX(EulerAnglesZYX{theta, 0.0f, 0.0f});
    const Vec3 out = controller.update(desired, Quaternion::Identity());

    // Exact closed form for this case (current == Identity): q_error == desired, and
    // FromEulerZYX's x component for a pure-roll rotation is exactly sin(theta/2).
    const float expected_x = 2.0f * kp * std::sin(theta * 0.5f);
    CHECK_NEAR(out.x, expected_x, 1e-5f, "positive roll error produces the expected positive roll rate");
    CHECK(out.x > 0.0f, "positive roll error produces a positive (not negative) roll rate");
    CHECK_NEAR(out.y, 0.0f, 1e-5f, "pure roll error produces ~zero pitch rate");
    CHECK_NEAR(out.z, 0.0f, 1e-5f, "pure roll error produces ~zero yaw rate");

    // Small-angle approximation check: kp*theta should be close to the exact value too (this is
    // the linearized law the file header documents, valid for small theta).
    CHECK_NEAR(out.x, kp * theta, 1e-3f, "small-angle approximation kp*theta tracks the exact value");

    // Opposite sign of desired roll flips the commanded direction.
    const Quaternion desired_neg = Quaternion::FromEulerZYX(EulerAnglesZYX{-theta, 0.0f, 0.0f});
    const Vec3 out_neg = controller.update(desired_neg, Quaternion::Identity());
    CHECK(out_neg.x < 0.0f, "negative roll error produces a negative roll rate");
}

static void test_single_axis_pitch_error()
{
    const float kp = 3.0f;
    AttitudeController controller(UnlimitedConfig(kp));

    const float theta = -0.08f; // negative pitch error this time, for variety
    const Quaternion desired = Quaternion::FromEulerZYX(EulerAnglesZYX{0.0f, theta, 0.0f});
    const Vec3 out = controller.update(desired, Quaternion::Identity());

    const float expected_y = 2.0f * kp * std::sin(theta * 0.5f);
    CHECK_NEAR(out.y, expected_y, 1e-5f, "negative pitch error produces the expected negative pitch rate");
    CHECK(out.y < 0.0f, "negative pitch error produces a negative (not positive) pitch rate");
    CHECK_NEAR(out.x, 0.0f, 1e-5f, "pure pitch error produces ~zero roll rate");
    CHECK_NEAR(out.z, 0.0f, 1e-5f, "pure pitch error produces ~zero yaw rate");
}

static void test_single_axis_yaw_error()
{
    const float kp = 2.5f;
    AttitudeController controller(UnlimitedConfig(kp));

    const float theta = 0.15f;
    const Quaternion desired = Quaternion::FromEulerZYX(EulerAnglesZYX{0.0f, 0.0f, theta});
    const Vec3 out = controller.update(desired, Quaternion::Identity());

    const float expected_z = 2.0f * kp * std::sin(theta * 0.5f);
    CHECK_NEAR(out.z, expected_z, 1e-5f, "positive yaw error produces the expected positive yaw rate");
    CHECK(out.z > 0.0f, "positive yaw error produces a positive yaw rate");
    CHECK_NEAR(out.x, 0.0f, 1e-5f, "pure yaw error produces ~zero roll rate");
    CHECK_NEAR(out.y, 0.0f, 1e-5f, "pure yaw error produces ~zero pitch rate");
}

static void test_combined_multi_axis_error()
{
    // Distinct per-axis gains, so a bug that mixed up which gain applies to which axis (or
    // swapped which error component drives which output axis) would show up as a wrong ratio
    // between the outputs, not just a wrong overall scale.
    AttitudeControllerConfig config;
    config.kp = Vec3(4.0f, 3.0f, 2.0f);
    config.rate_limit_radps = Vec3(0.0f, 0.0f, 0.0f); // unlimited, see UnlimitedConfig's comment
    AttitudeController controller(config);

    const float roll = 0.06f;
    const float pitch = -0.05f;
    const float yaw = 0.07f;
    const Quaternion desired = Quaternion::FromEulerZYX(EulerAnglesZYX{roll, pitch, yaw});
    const Vec3 out = controller.update(desired, Quaternion::Identity());

    // Exact closed form for FromEulerZYX's vector components (current == Identity, so q_error ==
    // desired exactly) — see quaternion.cpp's FromEulerZYX for the same formula.
    const float cr = std::cos(roll * 0.5f), sr = std::sin(roll * 0.5f);
    const float cp = std::cos(pitch * 0.5f), sp = std::sin(pitch * 0.5f);
    const float cy = std::cos(yaw * 0.5f), sy = std::sin(yaw * 0.5f);
    const float ex = sr * cp * cy - cr * sp * sy;
    const float ey = cr * sp * cy + sr * cp * sy;
    const float ez = cr * cp * sy - sr * sp * cy;

    CHECK_NEAR(out.x, 2.0f * config.kp.x * ex, 1e-5f, "combined error: roll axis matches exact closed form");
    CHECK_NEAR(out.y, 2.0f * config.kp.y * ey, 1e-5f, "combined error: pitch axis matches exact closed form");
    CHECK_NEAR(out.z, 2.0f * config.kp.z * ez, 1e-5f, "combined error: yaw axis matches exact closed form");

    // All three outputs should be individually nonzero and share the sign of their axis's error
    // angle (roll/yaw positive, pitch negative), confirming the axes weren't scrambled.
    CHECK(out.x > 0.0f, "combined error: roll rate has the expected (positive) sign");
    CHECK(out.y < 0.0f, "combined error: pitch rate has the expected (negative) sign");
    CHECK(out.z > 0.0f, "combined error: yaw rate has the expected (positive) sign");
}

static void test_feedforward_is_additive_at_zero_error()
{
    AttitudeController controller(UnlimitedConfig(4.0f));

    const Vec3 feedforward(0.5f, -0.3f, 0.2f);
    const Vec3 out = controller.update(Quaternion::Identity(), Quaternion::Identity(), feedforward);

    CHECK_NEAR(out.x, feedforward.x, 1e-6f, "feedforward passes through unchanged at zero error (roll)");
    CHECK_NEAR(out.y, feedforward.y, 1e-6f, "feedforward passes through unchanged at zero error (pitch)");
    CHECK_NEAR(out.z, feedforward.z, 1e-6f, "feedforward passes through unchanged at zero error (yaw)");
}

static void test_shortest_path_beyond_180_degrees()
{
    // Roll error of +200 degrees: the double-cover fix should make the controller command the
    // *shorter* -160 degree direction (negative roll rate), not naively chase the nominal +200
    // degree "long way around".
    AttitudeController controller(UnlimitedConfig(4.0f));

    const float theta = 200.0f * 3.14159265358979323846f / 180.0f;
    const Quaternion desired = Quaternion::FromEulerZYX(EulerAnglesZYX{theta, 0.0f, 0.0f});
    const Vec3 out = controller.update(desired, Quaternion::Identity());

    CHECK(out.x < 0.0f,
          "a +200 degree roll error commands a negative rate (shorter -160 degree direction)");

    // And the magnitude should match the shorter angle's exact closed form: after negating the
    // error quaternion (w was negative), the effective vector component is -sin(theta/2)... but
    // sin(theta/2) for theta=200deg (100deg half-angle) already exceeds w's sign threshold, so
    // compute the expected value the same way attitude_controller.cpp does: negate if w < 0.
    float ex = std::sin(theta * 0.5f);
    float ew = std::cos(theta * 0.5f);
    if (ew < 0.0f) {
        ex = -ex;
    }
    const float expected_x = 2.0f * 4.0f * ex;
    CHECK_NEAR(out.x, expected_x, 1e-4f, "shortest-path magnitude matches the sign-corrected closed form");
}

static void test_rate_limit_saturates()
{
    AttitudeControllerConfig config;
    config.kp = Vec3(10.0f, 10.0f, 10.0f); // deliberately large, to force saturation
    config.rate_limit_radps = Vec3(1.0f, 1.0f, 1.0f);
    AttitudeController controller(config);

    // A large-ish roll error alone; unclamped this would be 2*10*sin(0.25) ~= 4.9 rad/s, well
    // past the 1.0 rad/s limit.
    const float theta = 0.5f;
    const Quaternion desired = Quaternion::FromEulerZYX(EulerAnglesZYX{theta, 0.0f, 0.0f});
    const Vec3 out = controller.update(desired, Quaternion::Identity());

    CHECK_NEAR(out.x, 1.0f, 1e-6f, "roll rate is clamped to exactly the configured limit");
    CHECK_NEAR(out.y, 0.0f, 1e-6f, "unaffected pitch axis stays at zero despite roll saturating");
    CHECK_NEAR(out.z, 0.0f, 1e-6f, "unaffected yaw axis stays at zero despite roll saturating");

    // Negative error direction saturates to the negative limit.
    const Quaternion desired_neg = Quaternion::FromEulerZYX(EulerAnglesZYX{-theta, 0.0f, 0.0f});
    const Vec3 out_neg = controller.update(desired_neg, Quaternion::Identity());
    CHECK_NEAR(out_neg.x, -1.0f, 1e-6f, "roll rate is clamped to exactly the negative configured limit");

    // A small error, well under the limit, is not clamped.
    AttitudeControllerConfig small_config;
    small_config.kp = Vec3(4.0f, 4.0f, 4.0f);
    small_config.rate_limit_radps = Vec3(1.0f, 1.0f, 1.0f);
    AttitudeController small_controller(small_config);
    const Quaternion small_desired = Quaternion::FromEulerZYX(EulerAnglesZYX{0.05f, 0.0f, 0.0f});
    const Vec3 small_out = small_controller.update(small_desired, Quaternion::Identity());
    CHECK(small_out.x < 1.0f, "a small error's commanded rate stays below the configured limit");
    CHECK(small_out.x > 0.0f, "a small error still produces a nonzero commanded rate below the limit");
}

int main()
{
    test_zero_error_produces_zero_rate();
    test_single_axis_roll_error();
    test_single_axis_pitch_error();
    test_single_axis_yaw_error();
    test_combined_multi_axis_error();
    test_feedforward_is_additive_at_zero_error();
    test_shortest_path_beyond_180_degrees();
    test_rate_limit_saturates();

    std::printf("%d/%d checks passed\n", g_checks - g_failures, g_checks);
    return g_failures == 0 ? 0 : 1;
}
