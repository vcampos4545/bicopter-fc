#include "attitude_controller.h"

namespace bicopter {

namespace {
// Non-positive limit == "no limit configured for this axis" (see AttitudeControllerConfig's
// rate_limit_radps doc comment), matching Pid's philosophy of treating config fields as always
// meaningful rather than needing a separate enable flag.
float ClampSymmetric(float value, float limit)
{
    if (limit <= 0.0f) {
        return value;
    }
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}
} // namespace

AttitudeController::AttitudeController(const AttitudeControllerConfig& config) : config_(config)
{
}

Vec3 AttitudeController::update(const Quaternion& desired, const Quaternion& current,
                                 const Vec3& feedforward_radps) const
{
    // q_error = current^-1 * desired -- see the file header for the derivation and why the
    // opposite order silently flips control direction.
    Quaternion error = current.inverse() * desired;

    // Shortest-path fix for the quaternion double cover: q and -q are the same rotation, but
    // their vector parts have opposite sign. Forcing w >= 0 always picks the representation whose
    // rotation angle is in [0, 180] degrees. See the file header's "180-degree handling" note.
    if (error.w < 0.0f) {
        error.w = -error.w;
        error.x = -error.x;
        error.y = -error.y;
        error.z = -error.z;
    }

    // error.{x,y,z} ~= (angle_error/2) * axis for small angle_error; the factor of 2 recovers an
    // angle-times-axis rotation vector, then config_.kp is applied per axis. See the file header
    // for the full small-angle derivation and its accuracy limits.
    Vec3 commanded(2.0f * config_.kp.x * error.x, 2.0f * config_.kp.y * error.y,
                    2.0f * config_.kp.z * error.z);

    commanded = commanded + feedforward_radps;

    commanded.x = ClampSymmetric(commanded.x, config_.rate_limit_radps.x);
    commanded.y = ClampSymmetric(commanded.y, config_.rate_limit_radps.y);
    commanded.z = ClampSymmetric(commanded.z, config_.rate_limit_radps.z);

    return commanded;
}

} // namespace bicopter
