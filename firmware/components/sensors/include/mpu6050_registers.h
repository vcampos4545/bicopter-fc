// MPU6050 register map and bitfield constants, transcribed from the InvenSense MPU-6000/MPU-6050
// Register Map and Descriptions (RM-MPU-6000A-00) and Product Specification (PS-MPU-6000A-00).
// No I/O happens here; this header only names registers/bits used by mpu6050.c.
#ifndef BICOPTER_SENSORS_MPU6050_REGISTERS_H
#define BICOPTER_SENSORS_MPU6050_REGISTERS_H

// 7-bit I2C address: AD0 pin low (default, most breakout boards) or high.
#define MPU6050_I2C_ADDR_AD0_LOW  0x68
#define MPU6050_I2C_ADDR_AD0_HIGH 0x69

#define MPU6050_REG_SMPLRT_DIV    0x19
#define MPU6050_REG_CONFIG        0x1A // bits [2:0] = DLPF_CFG
#define MPU6050_REG_GYRO_CONFIG   0x1B // bits [4:3] = FS_SEL
#define MPU6050_REG_ACCEL_CONFIG  0x1C // bits [4:3] = AFS_SEL
#define MPU6050_REG_INT_PIN_CFG   0x37
#define MPU6050_REG_INT_ENABLE    0x38
#define MPU6050_REG_INT_STATUS    0x3A
#define MPU6050_REG_ACCEL_XOUT_H  0x3B // ACCEL_XOUT_H..GYRO_ZOUT_L is one contiguous 14-byte block
#define MPU6050_REG_TEMP_OUT_H    0x41
#define MPU6050_REG_GYRO_XOUT_H   0x43
#define MPU6050_REG_PWR_MGMT_1    0x6B
#define MPU6050_REG_WHO_AM_I      0x75

#define MPU6050_ACCEL_GYRO_BLOCK_LEN 14 // 3x accel (2B) + temp (2B) + 3x gyro (2B)

// INT_ENABLE / INT_STATUS bit 0: raw-data-ready interrupt.
#define MPU6050_BIT_DATA_RDY 0x01

// INT_PIN_CFG bit 5 (LATCH_INT_EN): interrupt pin stays high until INT_STATUS is read, rather
// than pulsing for 50us. Latching relaxes ISR-to-task-wake latency requirements since the line
// is still asserted whenever the task gets around to servicing it.
#define MPU6050_BIT_LATCH_INT_EN 0x20

// PWR_MGMT_1: wake from default sleep-on-reset state, clock off the X-gyro PLL (steadier than
// the internal 8MHz oscillator) per InvenSense's recommended startup sequence.
#define MPU6050_PWR1_CLKSEL_X_GYRO 0x01
#define MPU6050_PWR1_SLEEP_BIT     0x40

#define MPU6050_WHO_AM_I_VALUE 0x68

#endif // BICOPTER_SENSORS_MPU6050_REGISTERS_H
