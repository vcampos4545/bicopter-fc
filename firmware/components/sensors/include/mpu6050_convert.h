// Pure, hardware-independent MPU6050 logic: raw-register-to-SI conversion, calibration offset
// application, and stale/invalid-data detection. Nothing in this header or its .c file touches
// I2C, GPIO, or any ESP-IDF header, so it compiles and runs identically on the target and on the
// host (see tests/mpu6050_convert_test.c) — this is deliberate, per this milestone's brief:
// firmware-side I/O can't be unit tested without silicon, so the non-I/O logic is split out and
// tested in isolation instead of just asserted to work.
#ifndef BICOPTER_SENSORS_MPU6050_CONVERT_H
#define BICOPTER_SENSORS_MPU6050_CONVERT_H

#include <stdbool.h>
#include <stdint.h>

#include "imu.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MPU6050_ACCEL_RANGE_2G = 0,
    MPU6050_ACCEL_RANGE_4G,
    MPU6050_ACCEL_RANGE_8G,
    MPU6050_ACCEL_RANGE_16G,
} mpu6050_accel_range_t;

typedef enum {
    MPU6050_GYRO_RANGE_250DPS = 0,
    MPU6050_GYRO_RANGE_500DPS,
    MPU6050_GYRO_RANGE_1000DPS,
    MPU6050_GYRO_RANGE_2000DPS,
} mpu6050_gyro_range_t;

// DLPF_CFG values 0..6 (register CONFIG, per the datasheet's DLPF table). Value 0 leaves the
// gyro output rate at 8kHz (widest bandwidth, least filtering); 1..6 select progressively lower
// bandwidth/output rate in exchange for less noise.
typedef enum {
    MPU6050_DLPF_260HZ = 0,
    MPU6050_DLPF_184HZ,
    MPU6050_DLPF_94HZ,
    MPU6050_DLPF_44HZ,
    MPU6050_DLPF_21HZ,
    MPU6050_DLPF_10HZ,
    MPU6050_DLPF_5HZ,
} mpu6050_dlpf_bandwidth_t;

// Everything needed to interpret a raw sample and turn it into calibrated SI units. Deliberately
// excludes bus/pin config (that lives in mpu6050.h's hardware-facing config), so this struct can
// be constructed and exercised from a host test without any ESP-IDF dependency.
typedef struct {
    mpu6050_accel_range_t accel_range;
    mpu6050_gyro_range_t gyro_range;
    mpu6050_dlpf_bandwidth_t dlpf_bandwidth;   // informational for callers; register value is
                                                // derived by mpu6050.c from this at init time
    uint8_t sample_rate_div;                   // SMPLRT_DIV register value
    imu_vec3_t accel_offset_mps2;              // calibration: subtracted after conversion to SI
    imu_vec3_t gyro_offset_radps;              // calibration: subtracted after conversion to SI
} mpu6050_convert_config_t;

// Raw signed 16-bit register values in the order the datasheet lays them out in one contiguous
// burst read starting at ACCEL_XOUT_H.
typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t temp;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} mpu6050_raw_sample_t;

// LSB-per-g and LSB-per-(deg/s) sensitivity scale factors for each full-scale range, straight
// from the datasheet's sensitivity table.
float mpu6050_accel_lsb_per_g(mpu6050_accel_range_t range);
float mpu6050_gyro_lsb_per_dps(mpu6050_gyro_range_t range);

// Converts one raw axis reading to calibrated SI units: (raw / lsb_per_unit) * unit_to_si -
// offset. Pure functions so calibration math can be tested independent of register I/O.
float mpu6050_convert_accel_axis(int16_t raw, mpu6050_accel_range_t range, float offset_mps2);
float mpu6050_convert_gyro_axis(int16_t raw, mpu6050_gyro_range_t range, float offset_radps);

// Builds a full calibrated reading from a raw register sample. `drdy_signaled` and `i2c_ok` are
// plain bools rather than ESP-IDF's esp_err_t/queue-receive result so this function (and this
// whole module) stays free of ESP-IDF types; mpu6050.c maps its hardware results into these
// bools at the call site. `reading.valid` is false whenever either is false, in which case the
// accel/gyro fields are zeroed (not garbage from a failed transaction) and callers must not
// treat them as a sample.
imu_reading_t mpu6050_raw_to_reading(mpu6050_raw_sample_t raw,
                                      const mpu6050_convert_config_t *config,
                                      int64_t timestamp_us,
                                      bool drdy_signaled,
                                      bool i2c_ok);

// True if `now_us - last_valid_timestamp_us` exceeds `max_age_us` — for callers (e.g. an
// estimator) deciding whether a previously-valid reading is too old to keep using.
bool mpu6050_is_stale(int64_t last_valid_timestamp_us, int64_t now_us, int64_t max_age_us);

#ifdef __cplusplus
}
#endif

#endif // BICOPTER_SENSORS_MPU6050_CONVERT_H
