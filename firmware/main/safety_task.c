// SafetyTask - Milestone 6 shipped this as a stub that just re-affirmed the disarmed state every
// cycle (no failsafe condition existed to evaluate). Milestone 16 gives it real logic: the
// flight-mode state machine (flight_mode.h), arming preconditions (arming.h), and failsafe
// detection/response (failsafe.h) - all pure, host-tested modules from
// firmware/components/safety/ (see that component's file headers for why this milestone's logic
// lives there rather than in flight_core/safety/). This file's job is only to gather this cycle's
// real inputs and call those pure functions - the decision logic itself is not here, so it stays
// exactly as testable as the module headers promise.
//
// What's wired to genuinely live data this milestone, and what isn't yet (see docs/safety.md for
// the full breakdown and reasoning):
//   - IMU validity + raw gyro rate: REAL. xQueuePeek()s SensorTask's sample queue (the same queue
//     EstimatorTask consumes from - peeking never disturbs that single-consumer handoff and is
//     safe for any number of concurrent peekers) for the freshest imu_reading_t, whether that's
//     real hardware data or the honest all-invalid stub SensorTask publishes when
//     CONFIG_BICOPTER_SENSORS_ENABLED is off.
//   - Barometer validity: REAL, same queue peek.
//   - Radio arm command + link health: REAL. radio_state_get() (radio_state.h), published by
//     RadioTask every cycle - real hardware data when a radio backend is enabled, or the honest
//     "unavailable" default when neither is.
//   - Battery voltage/percent/thresholds: REAL when CONFIG_BICOPTER_BATTERY_ENABLED is set (no
//     battery hardware is chosen yet - see docs/hardware.md); battery_ok fails closed (blocks
//     arming) while disabled, same as every other missing-signal precondition.
//   - Estimator validity + attitude (tilt-from-level): NOT WIRED. No estimator runs in firmware
//     yet - AGENTS.md already notes EstimatorTask "still only logs/discards" every sample, and
//     wiring flight_core's C++ AttitudeEstimator into firmware needs the same not-yet-built
//     ESP-IDF-component/C++ boundary this project has deferred since Milestone 8. estimator_valid
//     is therefore hardcoded false here - never faked true - which means arming_preconditions_met()
//     can never return true in today's firmware regardless of every other condition. This is the
//     correct, safe behavior for a genuinely incomplete wiring (structurally blocking arming, not
//     silently skipping a check), not a bug; see docs/safety.md.
//   - Invalid-actuator-command defense-in-depth: not exercised - FlightControlTask doesn't call a
//     real control allocator in firmware yet (out of this milestone's scope, see AGENTS.md), so
//     there is no command to validate. actuator_command_valid is passed as true (nothing to
//     reject) - actuator_command_check.h's logic is implemented and host-tested independently,
//     ready for when that wiring lands.
//   - Task watchdog failure: not representable as an app-level failsafe_reason_t at all - see
//     docs/safety.md's "Task watchdog" section.
//
// Target rate: unchanged from Milestone 6 - 100Hz (SAFETY_TASK_PERIOD_MS = 10, see task_config.h)
// and highest priority of all six tasks.
#include <string.h>

#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "sdkconfig.h"

#include "arming.h"
#include "failsafe.h"
#include "flight_mode.h"

#include "radio_state.h"
#include "safety_state.h"
#include "safety_task.h"
#include "sensor_task.h"
#include "task_config.h"

// battery_convert.h (the pure voltage/percent/threshold math and battery_reading_t) is always
// included, even when battery monitoring is disabled - this file declares a battery_reading_t
// unconditionally below (zeroed, unread, when disabled) so the CONFIG_BICOPTER_BATTERY_ENABLED
// branches don't need their own separate variable declarations. battery.h (the ESP-IDF ADC driver
// I/O) stays behind the Kconfig guard, same as every other optional-hardware driver in this
// project.
#include "battery_convert.h"

#if CONFIG_BICOPTER_BATTERY_ENABLED
#include "battery.h"
#endif

static const char *TAG = "safety_task";

#define BICOPTER_DEG_TO_RAD(deg) ((deg) * 0.017453292519943295f)

#if CONFIG_BICOPTER_BATTERY_ENABLED
static battery_curve_point_t s_percent_curve[16];
static size_t s_percent_curve_len;

static void build_percent_curve(void)
{
    size_t base_len = 0;
    const battery_curve_point_t *base = battery_default_single_cell_curve(&base_len);
    size_t max_n = sizeof(s_percent_curve) / sizeof(s_percent_curve[0]);
    size_t n = (base_len < max_n) ? base_len : max_n;
    for (size_t i = 0; i < n; i++) {
        s_percent_curve[i].voltage_v = base[i].voltage_v * (float)CONFIG_BICOPTER_BATTERY_CELL_COUNT;
        s_percent_curve[i].percent = base[i].percent;
    }
    s_percent_curve_len = n;
}

// Real hardware init path. Reviewed against the ESP-IDF esp_adc oneshot/calibration API, not
// exercised: no physical voltage divider was available in this environment, and
// CONFIG_BICOPTER_BATTERY_ENABLED defaults to "n" for exactly that reason (see
// docs/hardware.md's TBD entry).
static esp_err_t battery_hw_init(battery_handle_t *out_handle)
{
    build_percent_curve();

    battery_config_t cfg = {
        .adc_unit = (CONFIG_BICOPTER_BATTERY_ADC_UNIT == 2) ? ADC_UNIT_2 : ADC_UNIT_1,
        .voltage_channel = (adc_channel_t)CONFIG_BICOPTER_BATTERY_VOLTAGE_ADC_CHANNEL,
        .atten = (adc_atten_t)CONFIG_BICOPTER_BATTERY_ADC_ATTENUATION,
        .convert = {
            .voltage_divider_ratio =
                (float)CONFIG_BICOPTER_BATTERY_VOLTAGE_DIVIDER_RATIO_MILLI / 1000.0f,
            .percent_curve = s_percent_curve,
            .percent_curve_len = s_percent_curve_len,
            .low_battery_voltage_v =
                ((float)CONFIG_BICOPTER_BATTERY_LOW_VOLTAGE_PER_CELL_MV *
                 (float)CONFIG_BICOPTER_BATTERY_CELL_COUNT) / 1000.0f,
            .critical_battery_voltage_v =
                ((float)CONFIG_BICOPTER_BATTERY_CRITICAL_VOLTAGE_PER_CELL_MV *
                 (float)CONFIG_BICOPTER_BATTERY_CELL_COUNT) / 1000.0f,
            .current_sense_scale_a_per_v = 0.0f,
        },
    };
#if CONFIG_BICOPTER_BATTERY_CURRENT_SENSE_ENABLED
    cfg.current_channel = (adc_channel_t)CONFIG_BICOPTER_BATTERY_CURRENT_ADC_CHANNEL;
    cfg.convert.current_sense_scale_a_per_v =
        (float)CONFIG_BICOPTER_BATTERY_CURRENT_SCALE_MILLIAMPS_PER_VOLT / 1000.0f;
#endif

    return battery_init(&cfg, out_handle);
}
#endif // CONFIG_BICOPTER_BATTERY_ENABLED

static failsafe_config_t build_failsafe_config(void)
{
    failsafe_config_t cfg = failsafe_default_config();

#if CONFIG_BICOPTER_BATTERY_ENABLED
    cfg.low_battery_voltage_v = ((float)CONFIG_BICOPTER_BATTERY_LOW_VOLTAGE_PER_CELL_MV *
                                  (float)CONFIG_BICOPTER_BATTERY_CELL_COUNT) / 1000.0f;
    cfg.critical_battery_voltage_v = ((float)CONFIG_BICOPTER_BATTERY_CRITICAL_VOLTAGE_PER_CELL_MV *
                                       (float)CONFIG_BICOPTER_BATTERY_CELL_COUNT) / 1000.0f;
#endif
    // Left at 0 (disabled) when battery monitoring is off - failsafe_evaluate() never checks
    // battery_voltage_v against these when the input voltage itself is <= 0 (see failsafe.c),
    // which is exactly what a caller with no battery ADC reports.

    cfg.max_tilt_angle_rad = (CONFIG_BICOPTER_SAFETY_MAX_TILT_ANGLE_DEG > 0)
                                  ? BICOPTER_DEG_TO_RAD((float)CONFIG_BICOPTER_SAFETY_MAX_TILT_ANGLE_DEG)
                                  : 0.0f;
    cfg.max_angular_rate_radps =
        (CONFIG_BICOPTER_SAFETY_MAX_ANGULAR_RATE_DPS > 0)
            ? BICOPTER_DEG_TO_RAD((float)CONFIG_BICOPTER_SAFETY_MAX_ANGULAR_RATE_DPS)
            : 0.0f;

    return cfg;
}

static flight_mode_request_t decode_mode_request(uint8_t raw_flight_mode)
{
    switch (raw_flight_mode) {
    case 1:
        return FLIGHT_MODE_REQUEST_STABILIZE;
    case 2:
        return FLIGHT_MODE_REQUEST_ALTITUDE_HOLD;
    default:
        return FLIGHT_MODE_REQUEST_ARMED_IDLE;
    }
}

void safety_task(void *pvParameters)
{
    safety_task_params_t *params = (safety_task_params_t *)pvParameters;

    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    failsafe_config_t fs_cfg = build_failsafe_config();

    float stationary_max_radps =
        (CONFIG_BICOPTER_SAFETY_STATIONARY_MAX_ANGULAR_RATE_DPS > 0)
            ? BICOPTER_DEG_TO_RAD((float)CONFIG_BICOPTER_SAFETY_STATIONARY_MAX_ANGULAR_RATE_DPS)
            : 0.0f;

#if CONFIG_BICOPTER_ALTITUDE_HOLD_ENABLED
    bool altitude_hold_enabled = true;
#else
    bool altitude_hold_enabled = false;
#endif

#if CONFIG_BICOPTER_BATTERY_ENABLED
    battery_handle_t battery = NULL;
    if (battery_hw_init(&battery) != ESP_OK) {
        ESP_LOGW(TAG, "battery_init failed; running without battery data (see docs/safety.md)");
        battery = NULL;
    } else {
        ESP_LOGI(TAG, "battery ADC initialized (unit=%d channel=%d)",
                 CONFIG_BICOPTER_BATTERY_ADC_UNIT, CONFIG_BICOPTER_BATTERY_VOLTAGE_ADC_CHANNEL);
    }
#else
    ESP_LOGI(TAG, "CONFIG_BICOPTER_BATTERY_ENABLED=n (no battery hardware chosen yet - see "
                  "docs/hardware.md); battery_ok will always be false, blocking arming");
#endif

    flight_mode_t mode = FLIGHT_MODE_BOOT;
    failsafe_reason_t last_failsafe_reason = FAILSAFE_REASON_NONE;

    uint32_t cycle = 0;
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        cycle++;

        // --- gather this cycle's inputs ---
        sensor_sample_t sample;
        memset(&sample, 0, sizeof(sample));
        bool have_sample = (params->sensor_sample_queue != NULL) &&
                            (xQueuePeek(params->sensor_sample_queue, &sample, 0) == pdTRUE);
        bool imu_valid = have_sample && sample.imu.valid;
        bool baro_valid = have_sample && sample.baro.valid;
        float gyro_x = have_sample ? sample.imu.gyro.x : 0.0f;
        float gyro_y = have_sample ? sample.imu.gyro.y : 0.0f;
        float gyro_z = have_sample ? sample.imu.gyro.z : 0.0f;

        radio_state_t radio;
        radio_state_get(&radio);
        bool radio_link_alive = radio.available && radio.health.link_alive;
        bool radio_arm_command = radio.available && radio.has_command && radio.command.arm;

        bool battery_have_reading = false;
        battery_reading_t battery_reading;
        memset(&battery_reading, 0, sizeof(battery_reading));
#if CONFIG_BICOPTER_BATTERY_ENABLED
        if (battery != NULL && battery_read(battery, &battery_reading) == ESP_OK) {
            battery_have_reading = true;
        }
#endif
        bool battery_ok = battery_have_reading && !battery_reading.low_battery;

        // Not wired in firmware yet - see this file's header. Never faked true.
        bool estimator_valid = false;
        float tilt_angle_rad = 0.0f;

        // --- arming preconditions ---
        arming_preconditions_t arming_in = {
            .imu_valid = imu_valid,
            .estimator_valid = estimator_valid,
            .radio_link_alive = radio_link_alive,
            .radio_arm_command = radio_arm_command,
            .battery_ok = battery_ok,
            .no_critical_errors = true, // no additional rollup source this milestone
            .is_stationary = arming_is_stationary(gyro_x, gyro_y, gyro_z, stationary_max_radps),
        };
        uint32_t blocking_mask = arming_blocking_mask(&arming_in);
        bool preconditions_met = (blocking_mask == 0);

        // --- failsafe evaluation ---
        float gyro_sq = gyro_x * gyro_x + gyro_y * gyro_y + gyro_z * gyro_z;
        failsafe_inputs_t fs_in = {
            .imu_valid = imu_valid,
            .estimator_valid = estimator_valid,
            .baro_valid = baro_valid,
            .radio_link_alive = radio_link_alive,
            .battery_voltage_v = battery_have_reading ? battery_reading.voltage_v : 0.0f,
            .tilt_angle_from_level_rad = tilt_angle_rad,
            .angular_rate_radps_sq = gyro_sq,
            .actuator_command_valid = true, // nothing to reject yet - see this file's header
        };
        failsafe_reason_t reason = failsafe_evaluate(&fs_in, &fs_cfg);
        bool failsafe_active = (reason != FAILSAFE_REASON_NONE);
        if (failsafe_active) {
            last_failsafe_reason = reason;
        }

        // --- flight-mode transition ---
        flight_mode_inputs_t fm_in = {
            .arm_command = radio_arm_command,
            .mode_request =
                (radio.available && radio.has_command) ? decode_mode_request(radio.command.flight_mode)
                                                          : FLIGHT_MODE_REQUEST_ARMED_IDLE,
            .arming_preconditions_met = preconditions_met,
            .failsafe_active = failsafe_active,
            .altitude_hold_enabled = altitude_hold_enabled,
            .critical_fault = false, // no detector wired this milestone - see flight_mode.h
        };
        mode = flight_mode_transition(mode, &fm_in);

        safety_state_t state = {
            .armed = flight_mode_is_armed(mode),
            .mode = mode,
            .last_failsafe_reason = last_failsafe_reason,
            .arming_blocking_mask = (mode == FLIGHT_MODE_DISARMED) ? blocking_mask : 0,
            .checked_at_us = esp_timer_get_time(),
        };
        safety_state_set(&state);
        xTaskNotifyGive(params->flight_control_handle);

        if ((cycle % 100) == 1) { // ~once/second at 100Hz
            ESP_LOGI(TAG,
                     "cycle=%lu mode=%s armed=%d failsafe=%s blocking_mask=0x%02lx",
                     (unsigned long)cycle, flight_mode_name(mode), state.armed,
                     failsafe_reason_name(last_failsafe_reason),
                     (unsigned long)state.arming_blocking_mask);
        }

        esp_task_wdt_reset();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SAFETY_TASK_PERIOD_MS));
    }
}
