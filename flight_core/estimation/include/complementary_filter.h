// Mahony-style nonlinear complementary-filter attitude estimator: gyro integration (via
// Quaternion::integrate, Milestone 7) corrected by an accelerometer-derived gravity-direction
// error fed back as a small-angle angular-rate correction, rather than blending Euler angles (the
// "classic" complementary filter) — see docs/estimation.md for the full derivation, why this
// design was picked over jumping straight to an EKF, and the accel-rejection behavior below.
#pragma once

#include "attitude_estimator.h"

namespace bicopter {

struct ComplementaryFilterConfig {
    // Static calibration offsets, subtracted from each raw sample before use — the same role as
    // Milestone 3's mpu6050_convert_config_t accel_offset_mps2/gyro_offset_radps fields, just
    // applied here instead of (or in addition to, if the HAL already zeroes them) the driver.
    Vec3 gyro_bias_radps = Vec3::Zero();
    Vec3 accel_bias_mps2 = Vec3::Zero();

    // Proportional gain: how strongly the accelerometer's gravity-direction error is fed back
    // into orientation propagation, in (rad/s of correction) per (unit of small-angle error).
    // This is the filter's gyro-vs-accel "crossover" knob — kp = 0 is pure gyro integration
    // (drifts freely), a large kp snaps orientation to the accelerometer every step (noisy, and
    // wrong under real acceleration). See docs/estimation.md for tuning guidance.
    float kp = 2.0f;

    // Integral gain: how fast the accelerometer error also adapts an online gyro-bias estimate
    // (rad/s^2 of bias correction per unit error), on top of the static gyro_bias_radps above.
    // 0 disables online bias adaptation (bias is then whatever gyro_bias_radps says and nothing
    // more).
    float ki = 0.0f;

    // Local gravity magnitude, m/s^2, used to judge whether an accel reading looks like "just
    // gravity."
    float gravity_mps2 = 9.80665f;

    // Accel-magnitude gating (the "unreasonable reading" handling from the milestone brief):
    // |accel| within accel_trust_band_g of 1g gets full correction weight (1.0); beyond
    // accel_reject_band_g of 1g gets zero weight (pure gyro integration that cycle); linearly
    // tapered in between. Both are fractions of gravity_mps2, e.g. 0.1 = "within 10% of 1g."
    // accel_reject_band_g must be > accel_trust_band_g.
    float accel_trust_band_g = 0.1f;
    float accel_reject_band_g = 0.3f;

    // A single update()'s dt is clamped to this many seconds before integrating, so a long gap
    // between valid samples (e.g. the IMU was stale for a while) can't produce one huge rotation
    // step when it recovers.
    float max_dt_s = 0.1f;
};

class ComplementaryFilterEstimator : public AttitudeEstimator {
public:
    explicit ComplementaryFilterEstimator(const ComplementaryFilterConfig& config = {});

    void reset(const Quaternion& initial_orientation) override;
    void update(const ImuSample& sample) override;
    AttitudeEstimate estimate() const override;

    // The accel-correction trust weight (0..1) applied on the most recent update() — exposed for
    // tests/telemetry to observe the high-acceleration-rejection behavior directly, not part of
    // the AttitudeEstimator interface since it's specific to this filter's implementation.
    float lastAccelWeight() const { return last_accel_weight_; }

private:
    ComplementaryFilterConfig config_;

    Quaternion orientation_;
    Vec3 angular_velocity_radps_;
    Vec3 gyro_bias_estimate_radps_; // online-adapted, on top of config_.gyro_bias_radps
    float last_accel_weight_ = 0.0f;

    bool have_timestamp_ = false;
    float last_timestamp_s_ = 0.0f;
    bool valid_ = false;
};

} // namespace bicopter
