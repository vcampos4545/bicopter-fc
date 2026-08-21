// Real automated tests for failsafe detection/response (failsafe.c): each condition firing
// correctly and NOT firing on the corresponding safe/normal input, the fixed priority order when
// multiple conditions are simultaneously true, and the configurable per-condition response table.
// Same hand-rolled harness as the rest of this project's host tests.
#include <stdio.h>
#include <string.h>

#include "failsafe.h"

static int g_failures = 0;
static int g_checks = 0;

#define CHECK(cond, msg)                                                                       \
    do {                                                                                        \
        g_checks++;                                                                             \
        if (!(cond)) {                                                                          \
            g_failures++;                                                                       \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, msg);                                \
        }                                                                                        \
    } while (0)

static failsafe_config_t test_config(void)
{
    failsafe_config_t cfg = failsafe_default_config();
    cfg.low_battery_voltage_v = 10.5f;
    cfg.critical_battery_voltage_v = 9.9f;
    cfg.max_tilt_angle_rad = 1.0f;         // ~57 degrees
    cfg.max_angular_rate_radps = 5.0f;
    return cfg;
}

static failsafe_inputs_t nominal_inputs(void)
{
    failsafe_inputs_t in;
    memset(&in, 0, sizeof(in));
    in.imu_valid = true;
    in.estimator_valid = true;
    in.baro_valid = true;
    in.radio_link_alive = true;
    in.battery_voltage_v = 12.0f;
    in.tilt_angle_from_level_rad = 0.05f;
    in.angular_rate_radps_sq = 0.01f;
    in.actuator_command_valid = true;
    return in;
}

static void test_nominal_input_triggers_nothing(void)
{
    failsafe_config_t cfg = test_config();
    failsafe_inputs_t in = nominal_inputs();
    CHECK(failsafe_evaluate(&in, &cfg) == FAILSAFE_REASON_NONE,
          "an entirely nominal input triggers no failsafe");
}

static void test_imu_failure(void)
{
    failsafe_config_t cfg = test_config();
    failsafe_inputs_t in = nominal_inputs();
    in.imu_valid = false;
    CHECK(failsafe_evaluate(&in, &cfg) == FAILSAFE_REASON_IMU_FAILURE,
          "invalid IMU triggers IMU_FAILURE");
}

static void test_estimator_failure(void)
{
    failsafe_config_t cfg = test_config();
    failsafe_inputs_t in = nominal_inputs();
    in.estimator_valid = false;
    CHECK(failsafe_evaluate(&in, &cfg) == FAILSAFE_REASON_ESTIMATOR_FAILURE,
          "invalid estimator (with a valid IMU) triggers ESTIMATOR_FAILURE");
}

static void test_excessive_attitude(void)
{
    failsafe_config_t cfg = test_config();
    failsafe_inputs_t in = nominal_inputs();

    in.tilt_angle_from_level_rad = cfg.max_tilt_angle_rad + 0.01f;
    CHECK(failsafe_evaluate(&in, &cfg) == FAILSAFE_REASON_EXCESSIVE_ATTITUDE,
          "tilt above the configured max triggers EXCESSIVE_ATTITUDE");

    in.tilt_angle_from_level_rad = cfg.max_tilt_angle_rad;
    CHECK(failsafe_evaluate(&in, &cfg) == FAILSAFE_REASON_NONE,
          "tilt exactly at the configured max does not trigger (strictly greater-than)");

    failsafe_config_t disabled_cfg = cfg;
    disabled_cfg.max_tilt_angle_rad = 0.0f;
    in.tilt_angle_from_level_rad = 3.0f; // ~172 degrees - would fail if the check were active
    CHECK(failsafe_evaluate(&in, &disabled_cfg) == FAILSAFE_REASON_NONE,
          "a non-positive max_tilt_angle_rad disables the excessive-attitude check");
}

static void test_excessive_angular_velocity(void)
{
    failsafe_config_t cfg = test_config();
    failsafe_inputs_t in = nominal_inputs();

    float over = cfg.max_angular_rate_radps + 1.0f;
    in.angular_rate_radps_sq = over * over;
    CHECK(failsafe_evaluate(&in, &cfg) == FAILSAFE_REASON_EXCESSIVE_ANGULAR_VELOCITY,
          "angular rate above the configured max triggers EXCESSIVE_ANGULAR_VELOCITY");

    in.angular_rate_radps_sq = cfg.max_angular_rate_radps * cfg.max_angular_rate_radps;
    CHECK(failsafe_evaluate(&in, &cfg) == FAILSAFE_REASON_NONE,
          "angular rate exactly at the configured max does not trigger");
}

static void test_radio_loss(void)
{
    failsafe_config_t cfg = test_config();
    failsafe_inputs_t in = nominal_inputs();
    in.radio_link_alive = false;
    CHECK(failsafe_evaluate(&in, &cfg) == FAILSAFE_REASON_RADIO_LOSS,
          "a dead radio link triggers RADIO_LOSS deterministically");
}

static void test_critical_battery(void)
{
    failsafe_config_t cfg = test_config();
    failsafe_inputs_t in = nominal_inputs();

    in.battery_voltage_v = cfg.critical_battery_voltage_v; // at threshold, inclusive
    CHECK(failsafe_evaluate(&in, &cfg) == FAILSAFE_REASON_CRITICAL_BATTERY,
          "battery voltage at/below the critical threshold triggers CRITICAL_BATTERY");

    in.battery_voltage_v = cfg.critical_battery_voltage_v + 0.5f;
    CHECK(failsafe_evaluate(&in, &cfg) != FAILSAFE_REASON_CRITICAL_BATTERY,
          "battery voltage above the critical threshold does not trigger CRITICAL_BATTERY");

    in.battery_voltage_v = 0.0f; // "no reading available" sentinel - see failsafe.h
    CHECK(failsafe_evaluate(&in, &cfg) == FAILSAFE_REASON_NONE,
          "a zero/absent battery voltage reading never triggers a battery failsafe (no data, not "
          "an empty pack)");
}

static void test_invalid_actuator_command(void)
{
    failsafe_config_t cfg = test_config();
    failsafe_inputs_t in = nominal_inputs();
    in.actuator_command_valid = false;
    CHECK(failsafe_evaluate(&in, &cfg) == FAILSAFE_REASON_INVALID_ACTUATOR_COMMAND,
          "an invalid actuator command triggers INVALID_ACTUATOR_COMMAND");
}

static void test_barometer_failure(void)
{
    failsafe_config_t cfg = test_config();
    failsafe_inputs_t in = nominal_inputs();
    in.baro_valid = false;
    CHECK(failsafe_evaluate(&in, &cfg) == FAILSAFE_REASON_BAROMETER_FAILURE,
          "an invalid barometer reading triggers BAROMETER_FAILURE");
}

static void test_low_battery(void)
{
    failsafe_config_t cfg = test_config();
    failsafe_inputs_t in = nominal_inputs();
    in.battery_voltage_v = cfg.low_battery_voltage_v; // at threshold, inclusive
    CHECK(failsafe_evaluate(&in, &cfg) == FAILSAFE_REASON_LOW_BATTERY,
          "battery voltage at/below the low threshold (but above critical) triggers LOW_BATTERY");
}

static void test_priority_order_reports_most_fundamental_failure(void)
{
    failsafe_config_t cfg = test_config();
    failsafe_inputs_t in = nominal_inputs();

    // Stack every condition on top of an invalid IMU - IMU_FAILURE must win, since a dead IMU
    // makes every other input (angular rate, tilt) meaningless to threshold-check in the first
    // place.
    in.imu_valid = false;
    in.estimator_valid = false;
    in.radio_link_alive = false;
    in.battery_voltage_v = 1.0f; // below critical
    in.baro_valid = false;
    in.actuator_command_valid = false;
    in.tilt_angle_from_level_rad = 10.0f;
    in.angular_rate_radps_sq = 1000.0f;
    CHECK(failsafe_evaluate(&in, &cfg) == FAILSAFE_REASON_IMU_FAILURE,
          "IMU_FAILURE outranks every other simultaneously-true condition");

    // With a valid IMU but invalid estimator, ESTIMATOR_FAILURE outranks radio/battery/etc.
    in.imu_valid = true;
    CHECK(failsafe_evaluate(&in, &cfg) == FAILSAFE_REASON_ESTIMATOR_FAILURE,
          "ESTIMATOR_FAILURE outranks radio/battery/baro/actuator/attitude/rate conditions");
}

static void test_default_config_responses(void)
{
    failsafe_config_t cfg = failsafe_default_config();
    failsafe_response_t response;

    CHECK(failsafe_get_response(&cfg, FAILSAFE_REASON_IMU_FAILURE, &response) &&
              response == FAILSAFE_RESPONSE_MOTOR_SHUTDOWN,
          "IMU_FAILURE defaults to MOTOR_SHUTDOWN");
    CHECK(failsafe_get_response(&cfg, FAILSAFE_REASON_CRITICAL_BATTERY, &response) &&
              response == FAILSAFE_RESPONSE_MOTOR_SHUTDOWN,
          "CRITICAL_BATTERY defaults to MOTOR_SHUTDOWN");
    CHECK(failsafe_get_response(&cfg, FAILSAFE_REASON_RADIO_LOSS, &response) &&
              response == FAILSAFE_RESPONSE_MOTOR_SHUTDOWN,
          "RADIO_LOSS defaults to MOTOR_SHUTDOWN");
    CHECK(failsafe_get_response(&cfg, FAILSAFE_REASON_BAROMETER_FAILURE, &response) &&
              response == FAILSAFE_RESPONSE_WARN_ONLY,
          "BAROMETER_FAILURE defaults to WARN_ONLY (not in the attitude-control loop)");
    CHECK(failsafe_get_response(&cfg, FAILSAFE_REASON_LOW_BATTERY, &response) &&
              response == FAILSAFE_RESPONSE_WARN_ONLY,
          "LOW_BATTERY defaults to WARN_ONLY (a pre-critical warning, not yet a controllability "
          "threat)");

    CHECK(!failsafe_get_response(&cfg, FAILSAFE_REASON_NONE, &response),
          "FAILSAFE_REASON_NONE has no response (returns false)");
    CHECK(!failsafe_get_response(&cfg, (failsafe_reason_t)9999, &response),
          "an out-of-range reason has no response (returns false)");
}

static void test_response_is_configurable(void)
{
    failsafe_config_t cfg = failsafe_default_config();
    cfg.response[FAILSAFE_REASON_LOW_BATTERY] = FAILSAFE_RESPONSE_MOTOR_SHUTDOWN;

    failsafe_response_t response;
    CHECK(failsafe_get_response(&cfg, FAILSAFE_REASON_LOW_BATTERY, &response) &&
              response == FAILSAFE_RESPONSE_MOTOR_SHUTDOWN,
          "per-condition response is caller-configurable, overriding the documented default");
}

static void test_reason_name_never_null(void)
{
    for (int r = FAILSAFE_REASON_NONE; r < FAILSAFE_REASON_COUNT; r++) {
        CHECK(failsafe_reason_name((failsafe_reason_t)r) != NULL,
              "failsafe_reason_name never returns NULL for a valid reason");
    }
}

static void test_null_input_fails_closed(void)
{
    failsafe_config_t cfg = test_config();
    failsafe_inputs_t in = nominal_inputs();
    CHECK(failsafe_evaluate(NULL, &cfg) != FAILSAFE_REASON_NONE,
          "a NULL inputs pointer never reports NONE (fails closed)");
    CHECK(failsafe_evaluate(&in, NULL) != FAILSAFE_REASON_NONE,
          "a NULL config pointer never reports NONE (fails closed)");
}

int main(void)
{
    test_nominal_input_triggers_nothing();
    test_imu_failure();
    test_estimator_failure();
    test_excessive_attitude();
    test_excessive_angular_velocity();
    test_radio_loss();
    test_critical_battery();
    test_invalid_actuator_command();
    test_barometer_failure();
    test_low_battery();
    test_priority_order_reports_most_fundamental_failure();
    test_default_config_responses();
    test_response_is_configurable();
    test_reason_name_never_null();
    test_null_input_fails_closed();

    printf("%d/%d checks passed\n", g_checks - g_failures, g_checks);
    return g_failures == 0 ? 0 : 1;
}
