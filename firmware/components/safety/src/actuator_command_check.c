#include "actuator_command_check.h"

#include <math.h>
#include <stddef.h>

static bool in_range(float v, float lo, float hi)
{
    if (!isfinite(v)) {
        return false;
    }
    float min_v = (lo <= hi) ? lo : hi;
    float max_v = (lo <= hi) ? hi : lo;
    return v >= min_v && v <= max_v;
}

bool actuator_command_is_valid(const actuator_command_t *cmd,
                                const actuator_command_limits_t *limits)
{
    if (cmd == NULL || limits == NULL) {
        return false;
    }
    return in_range(cmd->motor1_throttle, limits->min_throttle, limits->max_throttle) &&
           in_range(cmd->motor2_throttle, limits->min_throttle, limits->max_throttle) &&
           in_range(cmd->servo1_tilt_rad, limits->min_tilt_rad, limits->max_tilt_rad) &&
           in_range(cmd->servo2_tilt_rad, limits->min_tilt_rad, limits->max_tilt_rad);
}
