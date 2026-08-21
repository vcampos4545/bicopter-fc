#include "arming.h"

#include <stddef.h>

uint32_t arming_blocking_mask(const arming_preconditions_t *p)
{
    if (p == NULL) {
        // A missing input is treated as "everything blocked," never "nothing blocked" - see
        // arming_preconditions_met()'s same fail-closed rule.
        return ARMING_BLOCK_IMU | ARMING_BLOCK_ESTIMATOR | ARMING_BLOCK_RADIO_LINK |
               ARMING_BLOCK_RADIO_ARM_CMD | ARMING_BLOCK_BATTERY | ARMING_BLOCK_CRITICAL_ERROR |
               ARMING_BLOCK_NOT_STATIONARY;
    }

    uint32_t mask = 0;
    if (!p->imu_valid) {
        mask |= ARMING_BLOCK_IMU;
    }
    if (!p->estimator_valid) {
        mask |= ARMING_BLOCK_ESTIMATOR;
    }
    if (!p->radio_link_alive) {
        mask |= ARMING_BLOCK_RADIO_LINK;
    }
    if (!p->radio_arm_command) {
        mask |= ARMING_BLOCK_RADIO_ARM_CMD;
    }
    if (!p->battery_ok) {
        mask |= ARMING_BLOCK_BATTERY;
    }
    if (!p->no_critical_errors) {
        mask |= ARMING_BLOCK_CRITICAL_ERROR;
    }
    if (!p->is_stationary) {
        mask |= ARMING_BLOCK_NOT_STATIONARY;
    }
    return mask;
}

bool arming_preconditions_met(const arming_preconditions_t *p)
{
    return arming_blocking_mask(p) == 0;
}

bool arming_is_stationary(float gyro_x_radps, float gyro_y_radps, float gyro_z_radps,
                           float max_angular_rate_radps)
{
    if (max_angular_rate_radps <= 0.0f) {
        return true;
    }
    float mag_sq = gyro_x_radps * gyro_x_radps + gyro_y_radps * gyro_y_radps +
                   gyro_z_radps * gyro_z_radps;
    float threshold_sq = max_angular_rate_radps * max_angular_rate_radps;
    return mag_sq <= threshold_sq;
}
