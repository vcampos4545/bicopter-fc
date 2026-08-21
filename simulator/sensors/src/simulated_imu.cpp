#include "simulated_imu.h"

namespace bicopter {

SimulatedImu::SimulatedImu(const SimulatedImuConfig& config)
    : config_(config), rng_(config.seed), unit_normal_(0.0f, 1.0f)
{
}

Vec3 SimulatedImu::sampleNoise(const Vec3& stddev)
{
    // Three independent draws, one per axis -- no cross-axis noise coupling modeled, see the
    // header's file comment.
    return Vec3(unit_normal_(rng_) * stddev.x, unit_normal_(rng_) * stddev.y,
                unit_normal_(rng_) * stddev.z);
}

void SimulatedImu::reset()
{
    have_sampled_ = false;
    next_sample_time_s_ = 0.0f;
    rng_.seed(config_.seed);
}

std::optional<ImuSample> SimulatedImu::update(float t, const RigidBodyState& state,
                                               const Vec3& specific_force_body)
{
    if (have_sampled_ && t < next_sample_time_s_) {
        return std::nullopt;
    }
    have_sampled_ = true;
    next_sample_time_s_ = t + config_.sample_period_s;

    ImuSample sample;
    sample.gyro_radps =
        state.angular_velocity_radps + config_.gyro_bias_radps + sampleNoise(config_.gyro_noise_stddev_radps);
    sample.accel_mps2 =
        specific_force_body + config_.accel_bias_mps2 + sampleNoise(config_.accel_noise_stddev_mps2);
    sample.timestamp_s = t;
    sample.valid = true;
    return sample;
}

} // namespace bicopter
