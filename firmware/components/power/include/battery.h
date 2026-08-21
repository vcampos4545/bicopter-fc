// Battery ADC driver. Public API deliberately matches this project's other poll-style HAL shapes
// (mpu6050.h, bmp581.h): battery_read() returns one battery_reading_t (voltage, percent, optional
// current, threshold flags), no callbacks. Uses ESP-IDF's `esp_adc` oneshot driver plus a
// calibration scheme so raw ADC counts are converted to real millivolts before
// battery_convert.c's ESP-IDF-free math ever sees a value - ESP32 (the plain "esp32" target this
// project is pinned to, see AGENTS.md) only supports the line-fitting calibration scheme (see
// esp_adc/adc_cali_schemes.h), not the newer curve-fitting one available on later chips, so this
// driver uses adc_cali_create_scheme_line_fitting() specifically, not the more commonly-templated
// curve-fitting call.
//
// Current sensing is a second, optional ADC channel on the same unit (config->has_current_sense) -
// see docs/hardware.md and docs/safety.md for why this is supported but not required: no
// current-sense hardware (shunt resistor or hall-effect sensor) is chosen yet, same TBD status as
// the voltage divider itself.
#ifndef BICOPTER_POWER_BATTERY_H
#define BICOPTER_POWER_BATTERY_H

#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "hal/adc_types.h"

#include "battery_convert.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct battery_dev *battery_handle_t;

typedef struct {
    adc_unit_t adc_unit;
    adc_channel_t voltage_channel;
    adc_channel_t current_channel; // only read when convert.current_sense_scale_a_per_v != 0
    adc_atten_t atten;             // e.g. ADC_ATTEN_DB_12 for the ESP32's full ~0-3.3V input range

    battery_convert_config_t convert; // divider ratio, percent curve, thresholds, current scale
} battery_config_t;

// Initializes the ADC unit (oneshot mode), configures the voltage channel (and current channel,
// if configured), and creates a line-fitting calibration scheme for raw-to-millivolts conversion.
esp_err_t battery_init(const battery_config_t *config, battery_handle_t *out_handle);

// Poll-style read: one calibrated ADC conversion per configured channel, run through
// battery_convert_raw(). Must be called from task context (performs ADC transactions and, on
// failure, may log) - never from an ISR.
esp_err_t battery_read(battery_handle_t handle, battery_reading_t *out_reading);

// Releases the calibration scheme handle(s) and the ADC unit handle.
esp_err_t battery_deinit(battery_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif // BICOPTER_POWER_BATTERY_H
