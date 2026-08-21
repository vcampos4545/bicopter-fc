#include "rate_controller.h"

namespace bicopter {

RateController::RateController(const RateControllerConfig& config)
    : config_(config), roll_pid_(config.roll), pitch_pid_(config.pitch), yaw_pid_(config.yaw)
{
}

void RateController::reset()
{
    roll_pid_.reset();
    pitch_pid_.reset();
    yaw_pid_.reset();
}

Vec3 RateController::update(const Vec3& desired_radps, const Vec3& current_radps, float dt)
{
    const float roll_torque = roll_pid_.update(desired_radps.x, current_radps.x, dt);
    const float pitch_torque = pitch_pid_.update(desired_radps.y, current_radps.y, dt);
    const float yaw_torque = yaw_pid_.update(desired_radps.z, current_radps.z, dt);
    return Vec3(roll_torque, pitch_torque, yaw_torque);
}

} // namespace bicopter
