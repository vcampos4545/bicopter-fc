// Simulated Imu (Milestone 13): produces synthetic flight_core/estimation/'s ImuSample values
// from Milestone 9's RigidBodyState ground truth plus the body-frame specific force the dynamics
// model computes, so the estimator is fed noise-corrupted sensor data rather than perfect ground
// truth. See docs/simulation.md for exactly what's modeled (Gaussian noise, constant bias, a
// fixed sample rate) and what's deliberately simplified (no cross-axis coupling, no
// temperature/scale-factor error, no sensor delay beyond the sample-rate quantization itself).
//
// This does not implement docs/architecture.md's sketched `Imu` hardware-abstraction interface
// directly -- no concrete C++ `Imu` interface exists anywhere in this repository yet (only the
// firmware-side plain-C imu_reading_t local structs, per AGENTS.md's driver-testing-convention
// note) -- so this is a concrete producer of ImuSample, not an interface implementation. Wiring an
// actual `Imu` interface (shared between firmware/ and simulator/) is left for whichever future
// milestone first needs firmware and simulator to share a real polymorphic HAL type.
#pragma once

#include <cstdint>
#include <optional>
#include <random>

#include "attitude_estimator.h" // ImuSample
#include "rigid_body_state.h"
#include "vec3.h"

namespace bicopter {

struct SimulatedImuConfig {
    // Gaussian per-axis noise, one standard deviation, sampled independently every reading (no
    // cross-axis coupling, no time correlation between samples -- see docs/simulation.md).
    Vec3 gyro_noise_stddev_radps = Vec3::Zero();
    Vec3 accel_noise_stddev_mps2 = Vec3::Zero();

    // Constant per-axis bias, added on top of noise every reading -- models a real IMU's
    // uncalibrated (or imperfectly calibrated) zero-offset. Fixed for the whole run: does not
    // drift, and carries no temperature/scale-factor error model -- see docs/simulation.md.
    Vec3 gyro_bias_radps = Vec3::Zero();
    Vec3 accel_bias_mps2 = Vec3::Zero();

    // Sample period, seconds. Defaults to 2 ms / 500 Hz, matching SensorTask's real target rate
    // (firmware/main/task_config.h, docs/architecture.md's FreeRTOS task table).
    float sample_period_s = 0.002f;

    // Seed for the deterministic PRNG backing the Gaussian noise. Fixed by default so tests are
    // reproducible; a caller wanting a different noise realization across runs must pass a
    // different seed explicitly rather than relying on a time-based default.
    uint32_t seed = 1;
};

// Schedules and synthesizes noisy ImuSamples from ground truth on a fixed period. See the file
// header above for scope.
class SimulatedImu {
public:
    explicit SimulatedImu(const SimulatedImuConfig& config = {});

    // Advances the IMU's internal sample clock to sim time t (seconds). If at least
    // config.sample_period_s has elapsed since the last emitted sample (or this is the first call
    // since construction/reset), returns a new noisy ImuSample built from the given ground-truth
    // state/specific force; otherwise returns std::nullopt (no new reading yet, same as real
    // hardware's fixed output rate). Callers are expected to call this every physics tick with
    // the current sim time -- a physics tick shorter than sample_period_s is normal and expected.
    //
    // specific_force_body: body-frame specific force (m/s^2) -- what an accelerometer actually
    // measures (thrust + drag, excluding gravity's own contribution). See docs/simulation.md for
    // the derivation from StateDerivative::acceleration_mps2.
    std::optional<ImuSample> update(float t, const RigidBodyState& state,
                                     const Vec3& specific_force_body);

    // Clears the sample schedule (next update() call always emits immediately) and reseeds the
    // PRNG from config.seed, so a fresh run starting from t=0 reproduces the identical noise
    // sequence regardless of how many samples a prior run already drew.
    void reset();

private:
    SimulatedImuConfig config_;
    std::mt19937 rng_;
    std::normal_distribution<float> unit_normal_;
    bool have_sampled_ = false;
    float next_sample_time_s_ = 0.0f;

    Vec3 sampleNoise(const Vec3& stddev);
};

} // namespace bicopter
