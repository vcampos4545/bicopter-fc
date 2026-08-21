#include "failsafe.h"

#include <stddef.h>

failsafe_config_t failsafe_default_config(void)
{
    failsafe_config_t cfg;
    for (int i = 0; i < FAILSAFE_REASON_COUNT; i++) {
        cfg.response[i] = FAILSAFE_RESPONSE_MOTOR_SHUTDOWN;
    }
    cfg.response[FAILSAFE_REASON_BAROMETER_FAILURE] = FAILSAFE_RESPONSE_WARN_ONLY;
    cfg.response[FAILSAFE_REASON_LOW_BATTERY] = FAILSAFE_RESPONSE_WARN_ONLY;

    cfg.low_battery_voltage_v = 0.0f;
    cfg.critical_battery_voltage_v = 0.0f;
    cfg.max_tilt_angle_rad = 0.0f;
    cfg.max_angular_rate_radps = 0.0f;
    return cfg;
}

failsafe_reason_t failsafe_evaluate(const failsafe_inputs_t *in, const failsafe_config_t *cfg)
{
    if (in == NULL || cfg == NULL) {
        // Fail closed: a caller that can't supply real inputs/config gets treated as if the most
        // fundamental sensor had failed, never as "nothing wrong."
        return FAILSAFE_REASON_IMU_FAILURE;
    }

    // Priority order (highest first): a condition whose own inputs would be meaningless under an
    // upstream failure is always checked after that failure, so exactly one, correctly-diagnosed
    // reason is ever reported even when multiple conditions are simultaneously true (e.g. a dead
    // IMU also reads a "stationary" zeroed gyro - IMU_FAILURE must win, not
    // EXCESSIVE_ANGULAR_VELOCITY's absence).
    if (!in->imu_valid) {
        return FAILSAFE_REASON_IMU_FAILURE;
    }
    if (!in->estimator_valid) {
        return FAILSAFE_REASON_ESTIMATOR_FAILURE;
    }
    if (cfg->max_tilt_angle_rad > 0.0f && in->tilt_angle_from_level_rad > cfg->max_tilt_angle_rad) {
        return FAILSAFE_REASON_EXCESSIVE_ATTITUDE;
    }
    if (cfg->max_angular_rate_radps > 0.0f) {
        float threshold_sq = cfg->max_angular_rate_radps * cfg->max_angular_rate_radps;
        if (in->angular_rate_radps_sq > threshold_sq) {
            return FAILSAFE_REASON_EXCESSIVE_ANGULAR_VELOCITY;
        }
    }
    if (!in->radio_link_alive) {
        return FAILSAFE_REASON_RADIO_LOSS;
    }
    if (in->battery_voltage_v > 0.0f && in->battery_voltage_v <= cfg->critical_battery_voltage_v) {
        return FAILSAFE_REASON_CRITICAL_BATTERY;
    }
    if (!in->actuator_command_valid) {
        return FAILSAFE_REASON_INVALID_ACTUATOR_COMMAND;
    }
    if (!in->baro_valid) {
        return FAILSAFE_REASON_BAROMETER_FAILURE;
    }
    if (in->battery_voltage_v > 0.0f && in->battery_voltage_v <= cfg->low_battery_voltage_v) {
        return FAILSAFE_REASON_LOW_BATTERY;
    }

    return FAILSAFE_REASON_NONE;
}

bool failsafe_get_response(const failsafe_config_t *cfg, failsafe_reason_t reason,
                            failsafe_response_t *out_response)
{
    if (cfg == NULL || out_response == NULL) {
        return false;
    }
    if (reason <= FAILSAFE_REASON_NONE || reason >= FAILSAFE_REASON_COUNT) {
        return false;
    }
    *out_response = cfg->response[reason];
    return true;
}

const char *failsafe_reason_name(failsafe_reason_t reason)
{
    switch (reason) {
    case FAILSAFE_REASON_NONE:
        return "NONE";
    case FAILSAFE_REASON_IMU_FAILURE:
        return "IMU_FAILURE";
    case FAILSAFE_REASON_ESTIMATOR_FAILURE:
        return "ESTIMATOR_FAILURE";
    case FAILSAFE_REASON_EXCESSIVE_ATTITUDE:
        return "EXCESSIVE_ATTITUDE";
    case FAILSAFE_REASON_EXCESSIVE_ANGULAR_VELOCITY:
        return "EXCESSIVE_ANGULAR_VELOCITY";
    case FAILSAFE_REASON_RADIO_LOSS:
        return "RADIO_LOSS";
    case FAILSAFE_REASON_CRITICAL_BATTERY:
        return "CRITICAL_BATTERY";
    case FAILSAFE_REASON_INVALID_ACTUATOR_COMMAND:
        return "INVALID_ACTUATOR_COMMAND";
    case FAILSAFE_REASON_BAROMETER_FAILURE:
        return "BAROMETER_FAILURE";
    case FAILSAFE_REASON_LOW_BATTERY:
        return "LOW_BATTERY";
    default:
        return "UNKNOWN";
    }
}
