// MPU6050 I2C driver. Public API deliberately matches the project's `Imu` HAL shape from
// docs/architecture.md: `mpu6050_read()` is a poll-style call returning one `imu_reading_t`
// (accel m/s^2, gyro rad/s, timestamp, validity), no callbacks. Internally it uses the MPU6050's
// data-ready interrupt (when a GPIO is configured) to know *when* to do the I2C read rather than
// polling status registers on a timer; see the ISR/task split explained above mpu6050_read()'s
// implementation in mpu6050.c and in docs/hardware.md.
//
// Only raw accelerometer/gyroscope registers are read — the DMP is never engaged. State
// estimation from these raw samples is a later milestone's job (flight_core/estimation/), not
// this driver's.
#ifndef BICOPTER_SENSORS_MPU6050_H
#define BICOPTER_SENSORS_MPU6050_H

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"

#include "imu.h"
#include "mpu6050_convert.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct mpu6050_dev *mpu6050_handle_t;

typedef struct {
    // I2C bus is caller-owned (created via i2c_new_master_bus() before calling mpu6050_init());
    // which GPIOs carry SDA/SCL is a board-level config this driver never sees directly.
    i2c_master_bus_handle_t i2c_bus;
    uint16_t i2c_address;   // MPU6050_I2C_ADDR_AD0_LOW/HIGH depending on the AD0 strap; board TBD
    uint32_t i2c_clock_hz;  // e.g. 400000 for fast mode

    // GPIO wired to the MPU6050's INT pin. Pass GPIO_NUM_NC (-1) to disable interrupt-driven
    // handoff entirely; mpu6050_read() then falls back to polling the INT_STATUS data-ready bit
    // once per call instead of waiting on the ISR-fed queue. Exact pin is a board-level choice
    // (TBD, see docs/hardware.md) so it is configured here, never hardcoded in this driver.
    gpio_num_t data_ready_gpio;

    // How long mpu6050_read() blocks waiting for a data-ready signal (interrupt or poll) before
    // giving up and returning an invalid/stale reading.
    int64_t read_timeout_ms;

    mpu6050_convert_config_t convert; // ranges, DLPF, sample-rate divider, calibration offsets
} mpu6050_config_t;

// Initializes the sensor: probes the I2C address, verifies WHO_AM_I, wakes it from its
// power-on-reset sleep state, writes the configured accel/gyro range, DLPF bandwidth, and
// sample-rate divider, and (if `data_ready_gpio` is configured) installs the data-ready ISR.
esp_err_t mpu6050_init(const mpu6050_config_t *config, mpu6050_handle_t *out_handle);

// Poll-style Imu.read(): blocks up to `read_timeout_ms` for a data-ready signal, then performs
// one I2C burst read of the accel/gyro registers and returns the calibrated reading. Must be
// called from task context (it performs I2C transactions and, on timeout, may log) — never from
// an ISR.
esp_err_t mpu6050_read(mpu6050_handle_t handle, imu_reading_t *out_reading);

// Releases the ISR handler (if installed) and the I2C device handle. Does not delete the I2C bus
// (caller-owned).
esp_err_t mpu6050_deinit(mpu6050_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif // BICOPTER_SENSORS_MPU6050_H
