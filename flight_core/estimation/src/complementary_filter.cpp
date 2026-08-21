#include "complementary_filter.h"

#include <cmath>

namespace bicopter {

namespace {
// Below this accel magnitude, the reading can't be normalized into a meaningful direction (a
// dead/disconnected sensor reporting all-zero, or a reading close enough to free-fall that "which
// way is gravity" is meaningless) — treated the same as a fully-rejected reading.
constexpr float kMinAccelNorm = 1e-3f;
} // namespace

ComplementaryFilterEstimator::ComplementaryFilterEstimator(const ComplementaryFilterConfig& config)
    : config_(config)
{
    reset(Quaternion::Identity());
}

void ComplementaryFilterEstimator::reset(const Quaternion& initial_orientation)
{
    orientation_ = initial_orientation.normalized();
    angular_velocity_radps_ = Vec3::Zero();
    gyro_bias_estimate_radps_ = Vec3::Zero();
    last_accel_weight_ = 0.0f;
    have_timestamp_ = false;
    last_timestamp_s_ = 0.0f;
    valid_ = false;
}

void ComplementaryFilterEstimator::update(const ImuSample& sample)
{
    if (!sample.valid) {
        // A stale/failed reading: don't integrate garbage, and don't advance last_timestamp_s_
        // either, so that when a valid sample eventually arrives, its dt spans the real elapsed
        // gap (bounded by max_dt_s below) rather than silently understating it. The previous
        // orientation/angular-velocity estimate is retained as the last known state, but no
        // longer marked fresh.
        valid_ = false;
        return;
    }

    if (!have_timestamp_) {
        // First valid sample only establishes the time base — there's no previous timestamp to
        // compute a dt against yet, so nothing to integrate this call. Still publish the
        // bias-corrected gyro so a caller reading estimate() immediately after the first sample
        // sees a real angular-velocity value.
        last_timestamp_s_ = sample.timestamp_s;
        have_timestamp_ = true;
        angular_velocity_radps_ =
            sample.gyro_radps - config_.gyro_bias_radps - gyro_bias_estimate_radps_;
        valid_ = true;
        return;
    }

    float dt = sample.timestamp_s - last_timestamp_s_;
    last_timestamp_s_ = sample.timestamp_s;
    if (dt <= 0.0f) {
        // Non-monotonic or duplicate timestamp: nothing meaningful to integrate this cycle, but
        // the estimator itself is still in a valid state.
        valid_ = true;
        return;
    }
    if (dt > config_.max_dt_s) {
        dt = config_.max_dt_s;
    }

    const Vec3 corrected_gyro =
        sample.gyro_radps - config_.gyro_bias_radps - gyro_bias_estimate_radps_;

    // Accel-magnitude gating: judge whether this reading looks like "just gravity" (vehicle
    // roughly stationary/unaccelerated) or "real acceleration" (e.g. thrust, a bump) that would
    // make gravity-direction correction actively wrong. See docs/estimation.md for why this
    // matters and ComplementaryFilterConfig's accel_trust_band_g/accel_reject_band_g for the
    // taper shape.
    const Vec3 accel = sample.accel_mps2 - config_.accel_bias_mps2;
    const float accel_norm = accel.length();

    float weight = 0.0f;
    if (accel_norm > kMinAccelNorm && config_.gravity_mps2 > kMinAccelNorm) {
        const float g_ratio = accel_norm / config_.gravity_mps2;
        const float deviation = std::fabs(g_ratio - 1.0f);
        if (deviation <= config_.accel_trust_band_g) {
            weight = 1.0f;
        } else if (deviation < config_.accel_reject_band_g) {
            const float span = config_.accel_reject_band_g - config_.accel_trust_band_g;
            weight = 1.0f - (deviation - config_.accel_trust_band_g) / span;
        } else {
            weight = 0.0f;
        }
    }
    last_accel_weight_ = weight;

    Vec3 correction_radps = Vec3::Zero();
    if (weight > 0.0f) {
        // Mahony-style error term: compare the measured gravity direction (normalized accel, body
        // frame) against the direction the current orientation estimate predicts gravity should
        // point in the body frame. cross() of the two unit vectors is the small-angle rotation
        // (body frame) that would rotate "expected" onto "measured" — exactly the correction
        // integrate() needs to nudge orientation_ toward agreeing with the accelerometer.
        //
        // World "down" is +Z (NED); a stationary accelerometer reads the reaction force against
        // gravity, i.e. "up" (-Z world). orientation_.inverse().rotate(...) expresses that
        // world-frame direction in the current body-frame estimate (world -> body).
        const Vec3 measured_dir = accel / accel_norm;
        const Vec3 expected_dir = orientation_.inverse().rotate(Vec3(0.0f, 0.0f, -1.0f));
        const Vec3 error = cross(measured_dir, expected_dir);

        correction_radps = error * (config_.kp * weight);
        gyro_bias_estimate_radps_ += error * (config_.ki * weight * dt);
    }

    orientation_ = orientation_.integrate(corrected_gyro + correction_radps, dt);
    // Published angular velocity is the bias-corrected gyro measurement itself — the accel
    // feedback term is an orientation-propagation artifact, not part of the vehicle's actual
    // angular rate, so it's deliberately excluded from what callers read as "angular velocity."
    angular_velocity_radps_ = corrected_gyro;
    valid_ = true;
}

AttitudeEstimate ComplementaryFilterEstimator::estimate() const
{
    AttitudeEstimate out;
    out.orientation = orientation_;
    out.angular_velocity_radps = angular_velocity_radps_;
    out.valid = valid_;
    return out;
}

} // namespace bicopter
