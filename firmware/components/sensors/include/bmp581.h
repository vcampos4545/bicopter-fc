// BMP581 I2C driver. Public API deliberately matches the project's `Barometer` HAL shape from
// docs/architecture.md: `bmp581_read()` is a poll-style call returning one `barometer_reading_t`
// (pressure Pa, temperature degC, altitude m, timestamp, validity), no callbacks. Unlike
// mpu6050.c, this driver does not use a data-ready interrupt: the BMP581 is left running in
// continuous (free-running) power mode after init, so it keeps sampling at its configured ODR in
// the background, and bmp581_read() just fetches whatever is currently in the DATA registers —
// appropriate since baro dynamics are far slower than attitude dynamics (see
// docs/architecture.md's FreeRTOS task table) and don't need the interrupt-driven handoff the
// MPU6050 driver uses to minimize sample latency. See docs/hardware.md for the full choice and
// reasoning, and why BMP581 was picked over BMP390 for this milestone.
#ifndef BICOPTER_SENSORS_BMP581_H
#define BICOPTER_SENSORS_BMP581_H

#include "driver/i2c_master.h"
#include "esp_err.h"

#include "barometer.h"
#include "bmp581_convert.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct bmp581_dev *bmp581_handle_t;

typedef struct {
    // I2C bus is caller-owned (created via i2c_new_master_bus() before calling bmp581_init());
    // which GPIOs carry SDA/SCL is a board-level config this driver never sees directly.
    i2c_master_bus_handle_t i2c_bus;
    uint16_t i2c_address;   // BMP581_I2C_ADDR_SDO_LOW/HIGH depending on the SDO strap; board TBD
    uint32_t i2c_clock_hz;  // e.g. 400000 for fast mode

    // Raw ODR_CONFIG ODR[4:0] register value written at init (sensor is always placed into
    // continuous/free-running power mode; see bmp581_registers.h for why this field is a raw
    // passthrough rather than a named-rate enum). BMP581_ODR_FASTEST (0x00) is a safe default.
    uint8_t odr_reg_value;

    bmp581_convert_config_t convert; // sea-level reference pressure, filter coefficient
} bmp581_config_t;

// Initializes the sensor: probes the I2C address, verifies CHIP_ID, and places it into
// continuous power mode at the configured ODR.
esp_err_t bmp581_init(const bmp581_config_t *config, bmp581_handle_t *out_handle);

// Poll-style Barometer.read(): performs one I2C burst read of the temperature/pressure data
// registers and returns the converted, filtered reading. Must be called from task context (it
// performs I2C transactions and, on failure, may log) — never from an ISR.
esp_err_t bmp581_read(bmp581_handle_t handle, barometer_reading_t *out_reading);

// Releases the I2C device handle. Does not delete the I2C bus (caller-owned).
esp_err_t bmp581_deinit(bmp581_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif // BICOPTER_SENSORS_BMP581_H
