// MPU6050 driver implementation.
//
// ISR/task split (see docs/architecture.md "Interrupt context vs. task context" and
// docs/hardware.md for why interrupt-driven handoff was chosen over pure polling for this
// driver): the MPU6050's INT pin, when configured, is wired to a GPIO interrupt. The ISR
// (mpu6050_isr, IRAM_ATTR) does exactly two things: read the monotonic timestamp
// (esp_timer_get_time(), safe from ISR context — no bus access, no allocation) and hand it to
// task context via a length-1 FreeRTOS queue with xQueueOverwriteFromISR(). It performs no I2C
// transaction, no logging, and no heap allocation. All of that — the actual register burst read,
// unit conversion, and error handling — happens in mpu6050_read(), which runs in the calling
// task's context and blocks on that queue until the ISR posts a timestamp or `read_timeout_ms`
// elapses. This keeps the ISR minimal per standard FreeRTOS ISR discipline while still gating
// I2C reads on the sensor's own data-ready signal instead of a blind polling timer.
//
// If no data-ready GPIO is configured, mpu6050_read() instead polls the INT_STATUS register's
// data-ready bit once per call (a normal I2C transaction, safe in task context) before doing the
// burst read.

#include "mpu6050.h"

#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "mpu6050_registers.h"

static const char *TAG = "mpu6050";

struct mpu6050_dev {
    i2c_master_dev_handle_t i2c_dev;
    gpio_num_t data_ready_gpio;
    int64_t read_timeout_ms;
    mpu6050_convert_config_t convert;
    QueueHandle_t drdy_queue; // length 1, holds the ISR-captured timestamp (int64_t); NULL if polling
};

static void IRAM_ATTR mpu6050_isr(void *arg)
{
    struct mpu6050_dev *dev = (struct mpu6050_dev *)arg;
    int64_t now_us = esp_timer_get_time();
    BaseType_t higher_priority_task_woken = pdFALSE;
    xQueueOverwriteFromISR(dev->drdy_queue, &now_us, &higher_priority_task_woken);
    if (higher_priority_task_woken) {
        portYIELD_FROM_ISR();
    }
}

static esp_err_t mpu6050_write_reg(i2c_master_dev_handle_t i2c_dev, uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = { reg, value };
    return i2c_master_transmit(i2c_dev, buf, sizeof(buf), 1000);
}

static esp_err_t mpu6050_read_regs(i2c_master_dev_handle_t i2c_dev, uint8_t reg,
                                    uint8_t *out_buf, size_t len)
{
    return i2c_master_transmit_receive(i2c_dev, &reg, 1, out_buf, len, 1000);
}

// DLPF_CFG occupies CONFIG[2:0] and is numerically identical to mpu6050_dlpf_bandwidth_t's
// enumerators, but this indirection keeps the register encoding out of the pure convert module
// and makes the correspondence explicit rather than relying on enum values matching by accident.
static uint8_t mpu6050_dlpf_cfg_bits(mpu6050_dlpf_bandwidth_t dlpf)
{
    switch (dlpf) {
    case MPU6050_DLPF_260HZ: return 0;
    case MPU6050_DLPF_184HZ: return 1;
    case MPU6050_DLPF_94HZ:  return 2;
    case MPU6050_DLPF_44HZ:  return 3;
    case MPU6050_DLPF_21HZ:  return 4;
    case MPU6050_DLPF_10HZ:  return 5;
    case MPU6050_DLPF_5HZ:   return 6;
    default:                 return 0;
    }
}

esp_err_t mpu6050_init(const mpu6050_config_t *config, mpu6050_handle_t *out_handle)
{
    if (config == NULL || out_handle == NULL || config->i2c_bus == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    struct mpu6050_dev *dev = calloc(1, sizeof(struct mpu6050_dev));
    if (dev == NULL) {
        return ESP_ERR_NO_MEM;
    }
    dev->data_ready_gpio = config->data_ready_gpio;
    dev->read_timeout_ms = config->read_timeout_ms;
    dev->convert = config->convert;

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = config->i2c_address,
        .scl_speed_hz = config->i2c_clock_hz,
    };
    esp_err_t err = i2c_master_bus_add_device(config->i2c_bus, &dev_cfg, &dev->i2c_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_master_bus_add_device failed: %s", esp_err_to_name(err));
        free(dev);
        return err;
    }

    uint8_t who_am_i = 0;
    err = mpu6050_read_regs(dev->i2c_dev, MPU6050_REG_WHO_AM_I, &who_am_i, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WHO_AM_I read failed: %s", esp_err_to_name(err));
        goto fail;
    }
    if (who_am_i != MPU6050_WHO_AM_I_VALUE) {
        ESP_LOGE(TAG, "WHO_AM_I mismatch: got 0x%02x, expected 0x%02x", who_am_i,
                 MPU6050_WHO_AM_I_VALUE);
        err = ESP_ERR_NOT_FOUND;
        goto fail;
    }

    // Wake from power-on-reset sleep, clock off the X-gyro PLL per InvenSense's recommended
    // startup sequence (steadier than the internal 8MHz RC oscillator).
    err = mpu6050_write_reg(dev->i2c_dev, MPU6050_REG_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_X_GYRO);
    if (err != ESP_OK) {
        goto fail;
    }

    err = mpu6050_write_reg(dev->i2c_dev, MPU6050_REG_SMPLRT_DIV, config->convert.sample_rate_div);
    if (err != ESP_OK) {
        goto fail;
    }

    err = mpu6050_write_reg(dev->i2c_dev, MPU6050_REG_CONFIG,
                             mpu6050_dlpf_cfg_bits(config->convert.dlpf_bandwidth));
    if (err != ESP_OK) {
        goto fail;
    }

    err = mpu6050_write_reg(dev->i2c_dev, MPU6050_REG_GYRO_CONFIG,
                             (uint8_t)(config->convert.gyro_range << 3));
    if (err != ESP_OK) {
        goto fail;
    }

    err = mpu6050_write_reg(dev->i2c_dev, MPU6050_REG_ACCEL_CONFIG,
                             (uint8_t)(config->convert.accel_range << 3));
    if (err != ESP_OK) {
        goto fail;
    }

    if (dev->data_ready_gpio != GPIO_NUM_NC) {
        dev->drdy_queue = xQueueCreate(1, sizeof(int64_t));
        if (dev->drdy_queue == NULL) {
            err = ESP_ERR_NO_MEM;
            goto fail;
        }

        // Latch INT until INT_STATUS is read, then enable the data-ready interrupt.
        err = mpu6050_write_reg(dev->i2c_dev, MPU6050_REG_INT_PIN_CFG, MPU6050_BIT_LATCH_INT_EN);
        if (err != ESP_OK) {
            goto fail;
        }
        err = mpu6050_write_reg(dev->i2c_dev, MPU6050_REG_INT_ENABLE, MPU6050_BIT_DATA_RDY);
        if (err != ESP_OK) {
            goto fail;
        }

        gpio_config_t io_conf = {
            .pin_bit_mask = 1ULL << dev->data_ready_gpio,
            .mode = GPIO_MODE_INPUT,
            .intr_type = GPIO_INTR_POSEDGE, // INT_PIN_CFG default: active-high, push-pull
        };
        err = gpio_config(&io_conf);
        if (err != ESP_OK) {
            goto fail;
        }

        err = gpio_install_isr_service(0);
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) { // INVALID_STATE = already installed
            goto fail;
        }
        err = gpio_isr_handler_add(dev->data_ready_gpio, mpu6050_isr, dev);
        if (err != ESP_OK) {
            goto fail;
        }
    }

    *out_handle = dev;
    return ESP_OK;

fail:
    if (dev->drdy_queue != NULL) {
        vQueueDelete(dev->drdy_queue);
    }
    i2c_master_bus_rm_device(dev->i2c_dev);
    free(dev);
    return err;
}

// Task-context fallback used when no data-ready GPIO is configured: poll INT_STATUS once. This
// is an ordinary I2C transaction (not an ISR), so it's only ever called from mpu6050_read().
static esp_err_t mpu6050_poll_data_ready(mpu6050_handle_t dev, bool *out_ready)
{
    uint8_t status = 0;
    esp_err_t err = mpu6050_read_regs(dev->i2c_dev, MPU6050_REG_INT_STATUS, &status, 1);
    if (err != ESP_OK) {
        *out_ready = false;
        return err;
    }
    *out_ready = (status & MPU6050_BIT_DATA_RDY) != 0;
    return ESP_OK;
}

esp_err_t mpu6050_read(mpu6050_handle_t handle, imu_reading_t *out_reading)
{
    if (handle == NULL || out_reading == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    bool drdy_signaled = false;
    int64_t timestamp_us = esp_timer_get_time();

    if (handle->drdy_queue != NULL) {
        drdy_signaled = xQueueReceive(handle->drdy_queue, &timestamp_us,
                                       pdMS_TO_TICKS(handle->read_timeout_ms)) == pdTRUE;
        if (!drdy_signaled) {
            ESP_LOGW(TAG, "data-ready interrupt timed out after %lldms",
                     (long long)handle->read_timeout_ms);
        }
    } else {
        esp_err_t poll_err = mpu6050_poll_data_ready(handle, &drdy_signaled);
        if (poll_err != ESP_OK) {
            *out_reading = mpu6050_raw_to_reading((mpu6050_raw_sample_t){ 0 }, &handle->convert,
                                                   timestamp_us, false, false);
            return poll_err;
        }
        if (!drdy_signaled) {
            ESP_LOGW(TAG, "data not ready");
        }
    }

    if (!drdy_signaled) {
        *out_reading = mpu6050_raw_to_reading((mpu6050_raw_sample_t){ 0 }, &handle->convert,
                                               timestamp_us, false, true);
        return ESP_OK;
    }

    uint8_t buf[MPU6050_ACCEL_GYRO_BLOCK_LEN];
    esp_err_t err = mpu6050_read_regs(handle->i2c_dev, MPU6050_REG_ACCEL_XOUT_H, buf,
                                       sizeof(buf));
    bool i2c_ok = (err == ESP_OK);
    if (!i2c_ok) {
        ESP_LOGE(TAG, "burst read failed: %s", esp_err_to_name(err));
    }

    mpu6050_raw_sample_t raw = { 0 };
    if (i2c_ok) {
        raw.accel_x = (int16_t)((buf[0] << 8) | buf[1]);
        raw.accel_y = (int16_t)((buf[2] << 8) | buf[3]);
        raw.accel_z = (int16_t)((buf[4] << 8) | buf[5]);
        raw.temp    = (int16_t)((buf[6] << 8) | buf[7]);
        raw.gyro_x  = (int16_t)((buf[8] << 8) | buf[9]);
        raw.gyro_y  = (int16_t)((buf[10] << 8) | buf[11]);
        raw.gyro_z  = (int16_t)((buf[12] << 8) | buf[13]);
    }

    *out_reading = mpu6050_raw_to_reading(raw, &handle->convert, timestamp_us, drdy_signaled,
                                           i2c_ok);
    return i2c_ok ? ESP_OK : err;
}

esp_err_t mpu6050_deinit(mpu6050_handle_t handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (handle->data_ready_gpio != GPIO_NUM_NC) {
        gpio_isr_handler_remove(handle->data_ready_gpio);
    }
    if (handle->drdy_queue != NULL) {
        vQueueDelete(handle->drdy_queue);
    }
    esp_err_t err = i2c_master_bus_rm_device(handle->i2c_dev);
    free(handle);
    return err;
}
