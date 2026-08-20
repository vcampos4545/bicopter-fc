// BMP581 register map and bitfield constants, transcribed from the Bosch Sensortec BMP581
// digital pressure sensor datasheet (document BST-BMP581-DS004) as available to this driver's
// author. No physical BMP581 or datasheet PDF was available in this environment to cross-check
// byte-for-byte (see docs/hardware.md) — these addresses/values should be re-verified against the
// datasheet before flashing to real hardware, the same "register-map-reviewed, not
// hardware-verified" bar Milestone 3's MPU6050 driver was held to, but with less certainty here
// since the MPU6050 register map is far more widely cross-referenced than the BMP581's.
//
// No I/O happens here; this header only names registers/bits used by bmp581.c.
#ifndef BICOPTER_SENSORS_BMP581_REGISTERS_H
#define BICOPTER_SENSORS_BMP581_REGISTERS_H

// 7-bit I2C address: SDO pin low (default on most breakout boards) or high.
#define BMP581_I2C_ADDR_SDO_LOW  0x46
#define BMP581_I2C_ADDR_SDO_HIGH 0x47

#define BMP581_REG_CHIP_ID        0x01
#define BMP581_REG_TEMP_DATA_XLSB 0x1D // TEMP_DATA_XLSB..PRESS_DATA_MSB is one contiguous 6-byte
                                        // block: temperature (3B) immediately followed by
                                        // pressure (3B)
#define BMP581_REG_PRESS_DATA_XLSB 0x20
#define BMP581_REG_ODR_CONFIG      0x37 // bits [1:0] = PWR_MODE, bits [6:2] = ODR
#define BMP581_REG_CMD             0x7E

#define BMP581_TEMP_PRESS_BLOCK_LEN 6 // temp (3B) + press (3B)

#define BMP581_CHIP_ID_VALUE 0x50

#define BMP581_CMD_SOFT_RESET 0xB6

// ODR_CONFIG PWR_MODE[1:0]: continuous (free-running) mode makes the sensor keep sampling at the
// configured ODR in the background, so bmp581_read() can just fetch the latest DATA registers
// without needing a data-ready interrupt — appropriate here since baro dynamics are far slower
// than attitude dynamics (see docs/architecture.md's FreeRTOS task table), unlike the MPU6050
// where interrupt-driven handoff was worth the added complexity.
#define BMP581_PWR_MODE_STANDBY    0x00
#define BMP581_PWR_MODE_NORMAL     0x01
#define BMP581_PWR_MODE_FORCED     0x02
#define BMP581_PWR_MODE_CONTINUOUS 0x03

// ODR_CONFIG ODR[6:2]: output data rate selector. 0x00 selects the sensor's fastest rate (its
// power-on-reset default); this driver does not hardcode a full rate table (unlike
// mpu6050_dlpf_cfg_bits' DLPF table) because the exact ODR-value-to-Hz mapping is the piece of
// this register map this driver's author is least confident transcribing correctly without the
// datasheet PDF in hand. `bmp581_config_t.odr_reg_value` is a raw passthrough of this 5-bit field
// so a caller with the physical datasheet can select an exact rate; 0x00 (fastest) is a safe
// default that does not depend on that table being right.
#define BMP581_ODR_FASTEST 0x00

#endif // BICOPTER_SENSORS_BMP581_REGISTERS_H
