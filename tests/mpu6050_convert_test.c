// Real automated tests for the MPU6050 driver's pure logic: raw-register-to-SI conversion,
// calibration offset application, and stale/invalid-data detection (mpu6050_convert.c). No
// external test framework is pulled in (none is available offline in this environment); this is
// a small hand-rolled assert-and-report harness, run via `ctest` or the binary directly, exiting
// non-zero on any failure so it works as a real CI gate.
//
// What this file verifies vs. what remains unverified until real hardware is available: this
// covers the math/logic only (scale factors, offset subtraction, validity flagging, staleness
// comparison). It does NOT exercise I2C transactions, GPIO/ISR behavior, or actual sensor
// registers — those require real silicon (or at minimum QEMU with a simulated I2C peripheral,
// which was not attempted for this milestone). See docs/hardware.md for what's deferred.

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "mpu6050_convert.h"

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

#define CHECK_NEAR(a, b, tol, msg) CHECK(fabsf((a) - (b)) <= (tol), msg)

static void test_accel_scale_factors(void)
{
    CHECK_NEAR(mpu6050_accel_lsb_per_g(MPU6050_ACCEL_RANGE_2G), 16384.0f, 1e-6f, "2g scale");
    CHECK_NEAR(mpu6050_accel_lsb_per_g(MPU6050_ACCEL_RANGE_4G), 8192.0f, 1e-6f, "4g scale");
    CHECK_NEAR(mpu6050_accel_lsb_per_g(MPU6050_ACCEL_RANGE_8G), 4096.0f, 1e-6f, "8g scale");
    CHECK_NEAR(mpu6050_accel_lsb_per_g(MPU6050_ACCEL_RANGE_16G), 2048.0f, 1e-6f, "16g scale");
}

static void test_gyro_scale_factors(void)
{
    CHECK_NEAR(mpu6050_gyro_lsb_per_dps(MPU6050_GYRO_RANGE_250DPS), 131.0f, 1e-6f, "250dps scale");
    CHECK_NEAR(mpu6050_gyro_lsb_per_dps(MPU6050_GYRO_RANGE_500DPS), 65.5f, 1e-6f, "500dps scale");
    CHECK_NEAR(mpu6050_gyro_lsb_per_dps(MPU6050_GYRO_RANGE_1000DPS), 32.8f, 1e-6f, "1000dps scale");
    CHECK_NEAR(mpu6050_gyro_lsb_per_dps(MPU6050_GYRO_RANGE_2000DPS), 16.4f, 1e-6f, "2000dps scale");
}

static void test_accel_axis_conversion(void)
{
    // +1g on the +2g range should read back as standard gravity, zero calibration offset.
    float one_g = mpu6050_convert_accel_axis(16384, MPU6050_ACCEL_RANGE_2G, 0.0f);
    CHECK_NEAR(one_g, 9.80665f, 1e-3f, "16384 raw @ 2g range == 1g in m/s^2");

    // Negative raw (two's complement) should convert to negative SI value.
    float minus_one_g = mpu6050_convert_accel_axis(-16384, MPU6050_ACCEL_RANGE_2G, 0.0f);
    CHECK_NEAR(minus_one_g, -9.80665f, 1e-3f, "-16384 raw @ 2g range == -1g in m/s^2");

    // Wider range: 16384 raw @ 4g range should be 2g worth of acceleration.
    float two_g_on_4g_range = mpu6050_convert_accel_axis(16384, MPU6050_ACCEL_RANGE_4G, 0.0f);
    CHECK_NEAR(two_g_on_4g_range, 2.0f * 9.80665f, 1e-3f, "16384 raw @ 4g range == 2g");

    // Calibration offset is subtracted after conversion to SI units.
    float offset_applied = mpu6050_convert_accel_axis(16384, MPU6050_ACCEL_RANGE_2G, 0.5f);
    CHECK_NEAR(offset_applied, 9.80665f - 0.5f, 1e-3f, "accel calibration offset subtracted");

    // Zero raw with zero offset must be exactly zero.
    CHECK_NEAR(mpu6050_convert_accel_axis(0, MPU6050_ACCEL_RANGE_2G, 0.0f), 0.0f, 1e-6f,
               "zero raw accel converts to zero");
}

static void test_gyro_axis_conversion(void)
{
    // 131 raw @ 250dps range == 1 deg/s == pi/180 rad/s.
    float one_dps = mpu6050_convert_gyro_axis(131, MPU6050_GYRO_RANGE_250DPS, 0.0f);
    CHECK_NEAR(one_dps, 0.017453293f, 1e-4f, "131 raw @ 250dps range == 1 deg/s in rad/s");

    float minus_one_dps = mpu6050_convert_gyro_axis(-131, MPU6050_GYRO_RANGE_250DPS, 0.0f);
    CHECK_NEAR(minus_one_dps, -0.017453293f, 1e-4f, "-131 raw @ 250dps range == -1 deg/s in rad/s");

    // Calibration offset is subtracted after conversion to SI units.
    float offset_applied = mpu6050_convert_gyro_axis(131, MPU6050_GYRO_RANGE_250DPS, 0.01f);
    CHECK_NEAR(offset_applied, 0.017453293f - 0.01f, 1e-4f, "gyro calibration offset subtracted");

    CHECK_NEAR(mpu6050_convert_gyro_axis(0, MPU6050_GYRO_RANGE_250DPS, 0.0f), 0.0f, 1e-6f,
               "zero raw gyro converts to zero");
}

static mpu6050_convert_config_t default_config(void)
{
    mpu6050_convert_config_t config;
    memset(&config, 0, sizeof(config));
    config.accel_range = MPU6050_ACCEL_RANGE_2G;
    config.gyro_range = MPU6050_GYRO_RANGE_250DPS;
    config.dlpf_bandwidth = MPU6050_DLPF_44HZ;
    config.sample_rate_div = 0;
    return config;
}

static void test_raw_to_reading_valid(void)
{
    mpu6050_convert_config_t config = default_config();
    mpu6050_raw_sample_t raw = {
        .accel_x = 16384, .accel_y = 0, .accel_z = 0,
        .temp = 0,
        .gyro_x = 0, .gyro_y = 131, .gyro_z = 0,
    };

    imu_reading_t reading = mpu6050_raw_to_reading(raw, &config, 12345, true, true);

    CHECK(reading.valid, "reading valid when drdy_signaled && i2c_ok");
    CHECK(reading.timestamp_us == 12345, "timestamp passed through unchanged");
    CHECK_NEAR(reading.accel.x, 9.80665f, 1e-3f, "accel.x converted from raw");
    CHECK_NEAR(reading.accel.y, 0.0f, 1e-6f, "accel.y zero when raw is zero");
    CHECK_NEAR(reading.gyro.y, 0.017453293f, 1e-4f, "gyro.y converted from raw");
    CHECK_NEAR(reading.gyro.x, 0.0f, 1e-6f, "gyro.x zero when raw is zero");
}

static void test_raw_to_reading_invalid_on_no_drdy(void)
{
    mpu6050_convert_config_t config = default_config();
    mpu6050_raw_sample_t raw = { .accel_x = 16384, .gyro_y = 131 };

    imu_reading_t reading = mpu6050_raw_to_reading(raw, &config, 999, /*drdy_signaled=*/false,
                                                     /*i2c_ok=*/true);

    CHECK(!reading.valid, "reading invalid when data-ready never signaled (stale/timeout)");
    CHECK_NEAR(reading.accel.x, 0.0f, 1e-6f, "accel zeroed, not garbage, when invalid");
    CHECK_NEAR(reading.gyro.y, 0.0f, 1e-6f, "gyro zeroed, not garbage, when invalid");
    CHECK(reading.timestamp_us == 999, "timestamp still set on an invalid reading");
}

static void test_raw_to_reading_invalid_on_i2c_failure(void)
{
    mpu6050_convert_config_t config = default_config();
    mpu6050_raw_sample_t raw = { .accel_x = 16384, .gyro_y = 131 };

    imu_reading_t reading = mpu6050_raw_to_reading(raw, &config, 999, /*drdy_signaled=*/true,
                                                     /*i2c_ok=*/false);

    CHECK(!reading.valid, "reading invalid when I2C transaction failed");
    CHECK_NEAR(reading.accel.x, 0.0f, 1e-6f, "accel zeroed, not garbage, on I2C failure");
}

static void test_staleness(void)
{
    CHECK(!mpu6050_is_stale(1000, 1500, 1000), "not stale when age < max_age");
    CHECK(!mpu6050_is_stale(1000, 2000, 1000), "not stale exactly at max_age boundary");
    CHECK(mpu6050_is_stale(1000, 2001, 1000), "stale when age > max_age");
    CHECK(!mpu6050_is_stale(1000, 1000, 1000), "not stale with zero elapsed time");
}

int main(void)
{
    test_accel_scale_factors();
    test_gyro_scale_factors();
    test_accel_axis_conversion();
    test_gyro_axis_conversion();
    test_raw_to_reading_valid();
    test_raw_to_reading_invalid_on_no_drdy();
    test_raw_to_reading_invalid_on_i2c_failure();
    test_staleness();

    printf("%d/%d checks passed\n", g_checks - g_failures, g_checks);
    return g_failures == 0 ? 0 : 1;
}
