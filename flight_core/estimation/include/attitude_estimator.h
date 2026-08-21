// Public interface for flight_core's attitude estimator. See docs/estimation.md for the chosen
// algorithm (a Mahony-style nonlinear complementary filter) and why an EKF wasn't the starting
// point; this header exists specifically so a future EKF-based estimator can implement
// AttitudeEstimator and be swapped in without changing EstimatorTask (firmware) or the
// simulator's call site (Milestone 9+) — both consume only this interface, never the concrete
// filter type.
//
// Scope (Milestone 8): attitude (orientation + angular velocity) only. Altitude/vertical-velocity
// estimation from the barometer is explicitly deferred — see docs/estimation.md — so ImuSample
// carries no barometer field yet; a later milestone extending this interface for altitude should
// do so without breaking existing AttitudeEstimator callers (e.g. a separate parallel interface
// or an additive field with a documented default).
#pragma once

#include "quaternion.h"
#include "vec3.h"

namespace bicopter {

// One IMU reading, lightly processed to flight_core's SI/body-frame convention (see README.md):
// accel in m/s^2, gyro in rad/s, both body frame; timestamp in seconds (monotonic, arbitrary
// epoch — only differences between consecutive samples matter). This is deliberately the same
// shape as firmware/components/sensors/include/imu.h's imu_reading_t, just with SI-seconds
// timestamps instead of microseconds and Vec3 instead of the plain-C imu_vec3_t — converting
// between the two (imu_reading_t.timestamp_us / 1e6f) is the hardware-abstraction boundary's job,
// not flight_core's (see AGENTS.md's units convention).
struct ImuSample {
    Vec3 accel_mps2;
    Vec3 gyro_radps;
    float timestamp_s = 0.0f;
    // Mirrors imu_reading_t.valid: false if the sample is stale or came from a failed read.
    // Estimators must not integrate an invalid sample's accel/gyro fields as if they were real
    // data.
    bool valid = true;
};

// Current best-estimate attitude state an estimator publishes.
struct AttitudeEstimate {
    // Body-to-world (NED) attitude quaternion — see docs/math.md's Quaternion convention.
    Quaternion orientation = Quaternion::Identity();
    // Bias-corrected body-frame angular velocity, rad/s (i.e. the gyro measurement with
    // configured/estimated bias removed — not including any correction term an implementation
    // internally feeds back into orientation propagation).
    Vec3 angular_velocity_radps = Vec3::Zero();
    // False before the first sample is processed, or whenever the most recent sample was
    // invalid/stale — i.e. "can this be trusted as fresh," not "is accel currently being
    // trusted for drift correction" (implementations may reduce or skip accel correction while
    // still reporting a valid gyro-propagated estimate; see docs/estimation.md).
    bool valid = false;
};

// Abstract attitude estimator. Callers (EstimatorTask on firmware, the simulator's control loop)
// depend only on this interface: feed samples via update() in timestamp order, read the latest
// fused state via estimate(). No implementation detail of a specific filter (complementary-filter
// gains, an EKF's covariance state, ...) is visible here.
class AttitudeEstimator {
public:
    virtual ~AttitudeEstimator() = default;

    // Resets internal state (including any adaptive bias estimate) and seeds the orientation.
    // Callers should call this once before the first update(), e.g. with Quaternion::Identity()
    // or a known starting attitude.
    virtual void reset(const Quaternion& initial_orientation) = 0;

    // Feeds one new IMU sample. Samples must be presented in non-decreasing timestamp_s order;
    // an implementation is free to ignore (but must not crash on) a non-monotonic timestamp.
    virtual void update(const ImuSample& sample) = 0;

    // Returns the current best-estimate attitude state. Cheap/const — safe to call every control
    // cycle without re-running any fusion math.
    virtual AttitudeEstimate estimate() const = 0;
};

} // namespace bicopter
