// Plain-C shape of the project's "Imu" hardware-abstraction reading, per docs/architecture.md
// (poll-style `read() -> { accel, gyro, timestamp }`). Deliberately free of ESP-IDF types so it
// compiles unmodified on the host (see tests/mpu6050_convert_test.c) and mirrors the type
// flight_core's own Imu interface will use once flight_core exists (milestone 7+). This header
// is the sensors component's local copy of that shape, not flight_core's eventual interface
// definition — flight_core is still an unimplemented stub as of this milestone.
#ifndef BICOPTER_SENSORS_IMU_H
#define BICOPTER_SENSORS_IMU_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Body frame: right-handed, X = forward, Y = right, Z = down (see AGENTS.md).
typedef struct {
    float x;
    float y;
    float z;
} imu_vec3_t;

typedef struct {
    imu_vec3_t accel;      // m/s^2, body frame
    imu_vec3_t gyro;       // rad/s, body frame
    int64_t timestamp_us;  // monotonic microseconds, esp_timer_get_time()-compatible epoch
    bool valid;            // false if stale/invalid: I2C failure or no data-ready within deadline
} imu_reading_t;

#ifdef __cplusplus
}
#endif

#endif // BICOPTER_SENSORS_IMU_H
