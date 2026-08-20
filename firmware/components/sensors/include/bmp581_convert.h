// Pure, hardware-independent BMP581 logic: raw-register-to-SI conversion, pressure smoothing, and
// pressure-to-altitude derivation. Nothing in this header or its .c file touches I2C or any
// ESP-IDF header, so it compiles and runs identically on the target and on the host (see
// tests/bmp581_convert_test.c) — same split as mpu6050_convert.h (Milestone 3): firmware-side I/O
// can't be unit tested without silicon, so the non-I/O logic is separated out and tested in
// isolation instead of just asserted to work.
//
// Unlike the MPU6050, the BMP581 outputs already-compensated pressure/temperature values
// directly from its data registers (no OTP trim-coefficient polynomial to apply) — see
// docs/hardware.md for why this part was chosen over the BMP390 for this milestone.
#ifndef BICOPTER_SENSORS_BMP581_CONVERT_H
#define BICOPTER_SENSORS_BMP581_CONVERT_H

#include <stdbool.h>
#include <stdint.h>

#include "barometer.h"

#ifdef __cplusplus
extern "C" {
#endif

// Everything needed to turn a raw sample into a calibrated, filtered reading. Deliberately
// excludes bus/pin config (that lives in bmp581.h's hardware-facing config), so this struct can
// be constructed and exercised from a host test without any ESP-IDF dependency.
typedef struct {
    float sea_level_pa;  // reference pressure for altitude derivation, Pa. 101325.0f is the ISA
                          // standard sea-level pressure; set to a locally-measured QNH for
                          // altitude relative to the current field instead of mean sea level.
    float filter_alpha;  // exponential-moving-average coefficient in [0, 1] applied to pressure
                          // before altitude conversion: filtered = alpha*raw + (1-alpha)*prev.
                          // 1.0 disables smoothing (each reading passes through unchanged); values
                          // closer to 0 smooth more heavily at the cost of slower response to real
                          // altitude changes. Values outside [0, 1] are clamped.
} bmp581_convert_config_t;

// Running filter state, carried across calls by the caller (one instance per sensor instance).
// Separate from bmp581_convert_config_t because it's mutable per-call state, not configuration.
typedef struct {
    float filtered_pressure_pa;
    bool initialized;  // false until the first sample seeds filtered_pressure_pa directly,
                        // avoiding a slow ramp-up from an arbitrary initial value (e.g. 0)
} bmp581_filter_state_t;

// Raw register values in the order the datasheet lays them out in one contiguous burst read
// starting at TEMP_DATA_XLSB (temperature then pressure — see bmp581_registers.h).
typedef struct {
    int32_t temperature_raw;  // 24-bit two's-complement, sign-extended to int32; LSB = 1/65536 degC
    uint32_t pressure_raw;    // 24-bit unsigned; LSB = 1/64 Pa
} bmp581_raw_sample_t;

// Sign-extends a 24-bit two's-complement value (as assembled from 3 raw bytes) to a signed int32.
int32_t bmp581_sign_extend_24(uint32_t raw24);

// Converts one raw register value to calibrated SI units. Pure functions so the scale factors can
// be tested independent of register I/O. No calibration offset is applied here (unlike the
// MPU6050): the BMP581 outputs already-compensated values, so there is no per-axis bias to
// subtract at this layer.
float bmp581_convert_temperature_c(int32_t raw_temp);
float bmp581_convert_pressure_pa(uint32_t raw_pressure);

// Applies the configured exponential-moving-average filter to one new raw pressure sample,
// updating `state` in place and returning the new filtered value. The first call after a
// zero-initialized state seeds the filter directly from `raw_pressure_pa` rather than smoothing
// from zero.
float bmp581_filter_apply(bmp581_filter_state_t *state, float alpha, float raw_pressure_pa);

// Standard barometric formula: altitude (m) of a given pressure relative to `sea_level_pa`.
// altitude = 44330 * (1 - (pressure / sea_level_pa) ^ (1/5.255)); positive above the reference.
float bmp581_pressure_to_altitude_m(float pressure_pa, float sea_level_pa);

// Builds a full reading from a raw register sample: converts pressure/temperature to SI, filters
// pressure via `filter_state` (updated in place), and derives altitude from the filtered
// pressure. `i2c_ok` is a plain bool rather than ESP-IDF's esp_err_t so this function (and this
// whole module) stays free of ESP-IDF types; bmp581.c maps its hardware result into this bool at
// the call site. `reading.valid` is false when `i2c_ok` is false, in which case all fields except
// timestamp are zeroed (not garbage from a failed transaction) and the filter state is left
// untouched (a failed read must not perturb the running average).
barometer_reading_t bmp581_raw_to_reading(bmp581_raw_sample_t raw,
                                           const bmp581_convert_config_t *config,
                                           bmp581_filter_state_t *filter_state,
                                           int64_t timestamp_us,
                                           bool i2c_ok);

// True if `now_us - last_valid_timestamp_us` exceeds `max_age_us` — for callers (e.g. an
// estimator) deciding whether a previously-valid reading is too old to keep using.
bool bmp581_is_stale(int64_t last_valid_timestamp_us, int64_t now_us, int64_t max_age_us);

#ifdef __cplusplus
}
#endif

#endif // BICOPTER_SENSORS_BMP581_CONVERT_H
