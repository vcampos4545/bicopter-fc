// Real automated tests for flight_core/estimation/complementary_filter.{h,cpp}: stationary
// convergence (including recovery from a bad initial attitude via accel-based drift correction),
// known constant-rotation gyro integration, noisy-input robustness, and the high-acceleration
// accel-rejection behavior from docs/estimation.md. Same hand-rolled assert-and-report harness as
// tests/quaternion_test.cpp.

#include <cmath>
#include <cstdio>

#include "complementary_filter.h"
#include "quaternion.h"
#include "vec3.h"

using bicopter::AttitudeEstimate;
using bicopter::ComplementaryFilterConfig;
using bicopter::ComplementaryFilterEstimator;
using bicopter::EulerAnglesZYX;
using bicopter::ImuSample;
using bicopter::Quaternion;
using bicopter::Vec3;

namespace {
constexpr float kPi = 3.14159265358979323846f;
constexpr float kGravity = 9.80665f;
// World "down" is +Z (NED, see docs/math.md); a stationary accelerometer reads the reaction force
// against gravity, i.e. "up" == -Z world.
const Vec3 kLevelAccel(0.0f, 0.0f, -kGravity);
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

// Angle (radians, >= 0) between the rotations represented by two quaternions, robust to the q /
// -q sign ambiguity (same physical rotation).
static float quat_angle_between(const Quaternion& a, const Quaternion& b)
{
    float d = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
    if (d > 1.0f) {
        d = 1.0f;
    } else if (d < -1.0f) {
        d = -1.0f;
    }
    return 2.0f * std::acos(std::fabs(d));
}

// Simple deterministic pseudo-noise (no <random> dependency, and reproducible across platforms):
// a sum of a few incommensurate sinusoids, bounded to roughly [-amplitude, amplitude].
static float deterministic_noise(int i, float amplitude)
{
    const float t = static_cast<float>(i);
    return amplitude * 0.5f *
           (std::sin(t * 0.37f) + std::sin(t * 1.71f + 1.0f) + std::sin(t * 0.09f + 2.0f)) / 1.5f;
}

static void test_stationary_convergence()
{
    // The vehicle is physically level and at rest: gyro reads ~0, accel reads pure gravity. The
    // estimator is deliberately seeded with a wrong (tilted) initial attitude to prove accel-based
    // correction actually pulls it back to the truth, rather than trivially staying wherever it
    // started.
    //
    // The initial error is roll/pitch only (no yaw): a gravity-only accelerometer observes the
    // body-frame gravity direction, a 2-DOF measurement, so it can correct roll/pitch (tilt
    // relative to gravity) but is fundamentally blind to rotation about the gravity axis itself
    // (yaw) - see docs/estimation.md. A small residual error is still expected even with zero
    // initial yaw, since composing a roll error with a pitch error is non-commutative and leaves a
    // second-order component along the (uncorrectable) gravity axis - the 2-degree tolerance below
    // accounts for that, not for incomplete convergence.
    ComplementaryFilterEstimator est;
    const EulerAnglesZYX bad_guess{8.0f * kPi / 180.0f, -6.0f * kPi / 180.0f, 0.0f};
    est.reset(Quaternion::FromEulerZYX(bad_guess));

    const float dt = 0.002f; // 500 Hz, matches SensorTask's period (docs/architecture.md)
    const int steps = 2000;  // 4 seconds - ample time for kp=2 to converge
    float t = 0.0f;
    for (int i = 0; i < steps; ++i) {
        t += dt;
        ImuSample sample;
        sample.gyro_radps = Vec3::Zero();
        sample.accel_mps2 = kLevelAccel;
        sample.timestamp_s = t;
        sample.valid = true;
        est.update(sample);
    }

    const AttitudeEstimate result = est.estimate();
    CHECK(result.valid, "stationary convergence: estimate is valid after real samples");

    const float angle_error = quat_angle_between(result.orientation, Quaternion::Identity());
    CHECK(angle_error < 2.0f * kPi / 180.0f,
          "stationary convergence: settles within 2 degrees of the true level attitude");
    CHECK_NEAR(result.orientation.norm(), 1.0f, 1e-4f,
               "stationary convergence: orientation quaternion stays unit norm");

    // Holding steady once converged: run more steps and confirm it doesn't drift away again.
    for (int i = 0; i < 500; ++i) {
        t += dt;
        ImuSample sample;
        sample.gyro_radps = Vec3::Zero();
        sample.accel_mps2 = kLevelAccel;
        sample.timestamp_s = t;
        est.update(sample);
    }
    const float angle_error_held = quat_angle_between(est.estimate().orientation, Quaternion::Identity());
    CHECK(angle_error_held < 2.0f * kPi / 180.0f,
          "stationary convergence: holds the correct attitude once converged");
}

static void test_known_constant_rotation()
{
    // Pure gyro-integration path: feed a constant body-rate about Z with no accel disturbance
    // (Vec3::Zero() accel falls below the minimum normalizable magnitude, so the accel-correction
    // weight is exactly 0 regardless of kp - see ComplementaryFilterEstimator::update). The result
    // should match Quaternion::integrate() directly, since that's what the estimator falls back to
    // with zero accel weight - this is the same property test as
    // tests/quaternion_test.cpp's test_integrate_constant_angular_velocity, run through the
    // estimator's public interface instead of the raw math primitive.
    ComplementaryFilterEstimator est;
    est.reset(Quaternion::Identity());

    const Vec3 omega(0.0f, 0.0f, kPi); // pi rad/s about Z
    const int steps = 1000;
    const float total_time = 1.0f; // 1 second -> 180 deg about Z
    const float dt = total_time / steps;

    // The very first update() call only establishes the time base (there is no previous timestamp
    // to compute a dt against yet, per the documented contract in attitude_estimator.h) and
    // integrates nothing. Prime it at t=0 so the timed loop below covers exactly total_time of
    // real integration, matching tests/quaternion_test.cpp's equivalent property test.
    {
        ImuSample priming;
        priming.gyro_radps = omega;
        priming.accel_mps2 = Vec3::Zero();
        priming.timestamp_s = 0.0f;
        est.update(priming);
    }

    float t = 0.0f;
    for (int i = 0; i < steps; ++i) {
        t += dt;
        ImuSample sample;
        sample.gyro_radps = omega;
        sample.accel_mps2 = Vec3::Zero();
        sample.timestamp_s = t;
        est.update(sample);
        CHECK_NEAR(est.lastAccelWeight(), 0.0f, 1e-6f,
                   "known rotation: zero-magnitude accel gets zero correction weight");
    }

    const Quaternion expected(0.0f, 0.0f, 0.0f, 1.0f); // 180 deg about Z
    const AttitudeEstimate result = est.estimate();
    CHECK(quat_angle_between(result.orientation, expected) < 1e-3f,
          "known constant rotation integrates to the expected attitude over time");

    const Vec3 av = result.angular_velocity_radps;
    CHECK_NEAR(av.x, omega.x, 1e-6f, "known rotation: published angular velocity x matches gyro");
    CHECK_NEAR(av.y, omega.y, 1e-6f, "known rotation: published angular velocity y matches gyro");
    CHECK_NEAR(av.z, omega.z, 1e-6f, "known rotation: published angular velocity z matches gyro");
}

static void test_noisy_input_convergence()
{
    // Stationary/level truth again, but this time both gyro and accel carry bounded deterministic
    // noise the way real sensor data would. The filter should stay close to the truth throughout
    // (not just at the end) rather than integrating the noise into a growing drift.
    ComplementaryFilterEstimator est;
    est.reset(Quaternion::Identity());

    const float dt = 0.002f;
    const int steps = 3000; // 6 seconds
    float t = 0.0f;
    float max_angle_error_second_half = 0.0f;

    for (int i = 0; i < steps; ++i) {
        t += dt;
        ImuSample sample;
        sample.gyro_radps = Vec3(deterministic_noise(i, 0.05f), deterministic_noise(i + 100, 0.05f),
                                  deterministic_noise(i + 200, 0.05f)); // +-0.05 rad/s gyro noise
        sample.accel_mps2 =
            kLevelAccel + Vec3(deterministic_noise(i, 0.4f), deterministic_noise(i + 300, 0.4f),
                                deterministic_noise(i + 400, 0.4f)); // +-0.4 m/s^2 accel noise
        sample.timestamp_s = t;
        est.update(sample);

        CHECK(std::isfinite(est.estimate().orientation.w), "noisy input: orientation stays finite");

        if (i > steps / 2) {
            const float err = quat_angle_between(est.estimate().orientation, Quaternion::Identity());
            if (err > max_angle_error_second_half) {
                max_angle_error_second_half = err;
            }
        }
    }

    CHECK_NEAR(est.estimate().orientation.norm(), 1.0f, 1e-3f,
               "noisy input: orientation quaternion stays unit norm despite noise");
    CHECK(max_angle_error_second_half < 10.0f * kPi / 180.0f,
          "noisy input: converges and stays within 10 degrees of truth over the back half of the "
          "run, rather than diverging");
}

static void test_high_acceleration_rejection()
{
    // A large, sustained non-gravity acceleration (e.g. the vehicle thrusting hard) should not be
    // blindly trusted as "which way is down": the estimator must reduce/skip the accel correction
    // rather than snapping orientation onto the bogus direction. Gyro reads zero throughout, so
    // with accel correction properly rejected the orientation must not move at all.
    ComplementaryFilterConfig config; // defaults: kp=2, trust band 0.1g, reject band 0.3g
    ComplementaryFilterEstimator est(config);
    est.reset(Quaternion::Identity());

    const Vec3 huge_lateral_accel(3.0f * kGravity, 0.0f, 0.0f); // 3g sideways - clearly not gravity
    const float dt = 0.002f;
    float t = 0.0f;
    for (int i = 0; i < 500; ++i) {
        t += dt;
        ImuSample sample;
        sample.gyro_radps = Vec3::Zero();
        sample.accel_mps2 = huge_lateral_accel;
        sample.timestamp_s = t;
        est.update(sample);

        CHECK_NEAR(est.lastAccelWeight(), 0.0f, 1e-6f,
                   "high acceleration: accel correction weight is fully rejected at 3g");
    }

    const float angle_drift = quat_angle_between(est.estimate().orientation, Quaternion::Identity());
    CHECK(angle_drift < 1e-3f,
          "high acceleration: orientation is untouched when accel correction is fully rejected "
          "and gyro reads zero");

    // Contrast case: the same magnitude of accel error, but small enough to fall inside the trust
    // band, DOES get corrected - confirms the gating logic (not just kp=0) is what's rejecting the
    // 3g case above.
    ComplementaryFilterEstimator est_small_error(config);
    const EulerAnglesZYX slight_tilt{5.0f * kPi / 180.0f, 0.0f, 0.0f};
    est_small_error.reset(Quaternion::FromEulerZYX(slight_tilt));
    t = 0.0f;
    for (int i = 0; i < 1000; ++i) {
        t += dt;
        ImuSample sample;
        sample.gyro_radps = Vec3::Zero();
        sample.accel_mps2 = kLevelAccel; // within the trust band of exactly 1g
        sample.timestamp_s = t;
        est_small_error.update(sample);
    }
    CHECK(est_small_error.lastAccelWeight() > 0.99f,
          "contrast case: a normal 1g reading gets full accel-correction weight");
    const float corrected_error =
        quat_angle_between(est_small_error.estimate().orientation, Quaternion::Identity());
    CHECK(corrected_error < 1.0f * kPi / 180.0f,
          "contrast case: a trusted accel reading does correct a slightly-wrong initial attitude");
}

int main()
{
    test_stationary_convergence();
    test_known_constant_rotation();
    test_noisy_input_convergence();
    test_high_acceleration_rejection();

    std::printf("%d/%d checks passed\n", g_checks - g_failures, g_checks);
    return g_failures == 0 ? 0 : 1;
}
