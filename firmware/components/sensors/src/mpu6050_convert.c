#include "mpu6050_convert.h"

#include <string.h>

#define MPU6050_STANDARD_GRAVITY_MPS2 9.80665f
#define MPU6050_DEG_TO_RAD 0.017453292519943295f

float mpu6050_accel_lsb_per_g(mpu6050_accel_range_t range)
{
    switch (range) {
    case MPU6050_ACCEL_RANGE_2G:  return 16384.0f;
    case MPU6050_ACCEL_RANGE_4G:  return 8192.0f;
    case MPU6050_ACCEL_RANGE_8G:  return 4096.0f;
    case MPU6050_ACCEL_RANGE_16G: return 2048.0f;
    default:                      return 16384.0f;
    }
}

float mpu6050_gyro_lsb_per_dps(mpu6050_gyro_range_t range)
{
    switch (range) {
    case MPU6050_GYRO_RANGE_250DPS:  return 131.0f;
    case MPU6050_GYRO_RANGE_500DPS:  return 65.5f;
    case MPU6050_GYRO_RANGE_1000DPS: return 32.8f;
    case MPU6050_GYRO_RANGE_2000DPS: return 16.4f;
    default:                         return 131.0f;
    }
}

float mpu6050_convert_accel_axis(int16_t raw, mpu6050_accel_range_t range, float offset_mps2)
{
    float g = (float)raw / mpu6050_accel_lsb_per_g(range);
    return g * MPU6050_STANDARD_GRAVITY_MPS2 - offset_mps2;
}

float mpu6050_convert_gyro_axis(int16_t raw, mpu6050_gyro_range_t range, float offset_radps)
{
    float dps = (float)raw / mpu6050_gyro_lsb_per_dps(range);
    return dps * MPU6050_DEG_TO_RAD - offset_radps;
}

imu_reading_t mpu6050_raw_to_reading(mpu6050_raw_sample_t raw,
                                      const mpu6050_convert_config_t *config,
                                      int64_t timestamp_us,
                                      bool drdy_signaled,
                                      bool i2c_ok)
{
    imu_reading_t reading;
    memset(&reading, 0, sizeof(reading));
    reading.timestamp_us = timestamp_us;
    reading.valid = drdy_signaled && i2c_ok;

    if (!reading.valid) {
        return reading;
    }

    reading.accel.x = mpu6050_convert_accel_axis(raw.accel_x, config->accel_range,
                                                  config->accel_offset_mps2.x);
    reading.accel.y = mpu6050_convert_accel_axis(raw.accel_y, config->accel_range,
                                                  config->accel_offset_mps2.y);
    reading.accel.z = mpu6050_convert_accel_axis(raw.accel_z, config->accel_range,
                                                  config->accel_offset_mps2.z);

    reading.gyro.x = mpu6050_convert_gyro_axis(raw.gyro_x, config->gyro_range,
                                                config->gyro_offset_radps.x);
    reading.gyro.y = mpu6050_convert_gyro_axis(raw.gyro_y, config->gyro_range,
                                                config->gyro_offset_radps.y);
    reading.gyro.z = mpu6050_convert_gyro_axis(raw.gyro_z, config->gyro_range,
                                                config->gyro_offset_radps.z);

    return reading;
}

bool mpu6050_is_stale(int64_t last_valid_timestamp_us, int64_t now_us, int64_t max_age_us)
{
    return (now_us - last_valid_timestamp_us) > max_age_us;
}
