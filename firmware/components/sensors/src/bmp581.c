// BMP581 driver implementation.
//
// No data-ready interrupt is used (see bmp581.h and docs/hardware.md for why): the sensor is
// placed into continuous power mode at init and left free-running, so bmp581_read() just performs
// one I2C burst read of the current temperature/pressure data registers on each call. All I2C
// transactions happen here, in task context, called from the calling task — never from an ISR.

#include "bmp581.h"

#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"

#include "bmp581_registers.h"

static const char *TAG = "bmp581";

struct bmp581_dev {
    i2c_master_dev_handle_t i2c_dev;
    bmp581_convert_config_t convert;
    bmp581_filter_state_t filter_state;
};

static esp_err_t bmp581_write_reg(i2c_master_dev_handle_t i2c_dev, uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = { reg, value };
    return i2c_master_transmit(i2c_dev, buf, sizeof(buf), 1000);
}

static esp_err_t bmp581_read_regs(i2c_master_dev_handle_t i2c_dev, uint8_t reg, uint8_t *out_buf,
                                   size_t len)
{
    return i2c_master_transmit_receive(i2c_dev, &reg, 1, out_buf, len, 1000);
}

esp_err_t bmp581_init(const bmp581_config_t *config, bmp581_handle_t *out_handle)
{
    if (config == NULL || out_handle == NULL || config->i2c_bus == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    struct bmp581_dev *dev = calloc(1, sizeof(struct bmp581_dev));
    if (dev == NULL) {
        return ESP_ERR_NO_MEM;
    }
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

    uint8_t chip_id = 0;
    err = bmp581_read_regs(dev->i2c_dev, BMP581_REG_CHIP_ID, &chip_id, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "CHIP_ID read failed: %s", esp_err_to_name(err));
        goto fail;
    }
    if (chip_id != BMP581_CHIP_ID_VALUE) {
        ESP_LOGE(TAG, "CHIP_ID mismatch: got 0x%02x, expected 0x%02x", chip_id,
                 BMP581_CHIP_ID_VALUE);
        err = ESP_ERR_NOT_FOUND;
        goto fail;
    }

    // ODR_CONFIG: ODR[6:2] = config->odr_reg_value, PWR_MODE[1:0] = continuous (free-running).
    uint8_t odr_config = (uint8_t)((config->odr_reg_value << 2) | BMP581_PWR_MODE_CONTINUOUS);
    err = bmp581_write_reg(dev->i2c_dev, BMP581_REG_ODR_CONFIG, odr_config);
    if (err != ESP_OK) {
        goto fail;
    }

    *out_handle = dev;
    return ESP_OK;

fail:
    i2c_master_bus_rm_device(dev->i2c_dev);
    free(dev);
    return err;
}

esp_err_t bmp581_read(bmp581_handle_t handle, barometer_reading_t *out_reading)
{
    if (handle == NULL || out_reading == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int64_t timestamp_us = esp_timer_get_time();

    uint8_t buf[BMP581_TEMP_PRESS_BLOCK_LEN];
    esp_err_t err = bmp581_read_regs(handle->i2c_dev, BMP581_REG_TEMP_DATA_XLSB, buf, sizeof(buf));
    bool i2c_ok = (err == ESP_OK);
    if (!i2c_ok) {
        ESP_LOGE(TAG, "burst read failed: %s", esp_err_to_name(err));
    }

    bmp581_raw_sample_t raw = { 0 };
    if (i2c_ok) {
        uint32_t temp_raw24 = ((uint32_t)buf[2] << 16) | ((uint32_t)buf[1] << 8) | buf[0];
        uint32_t press_raw24 = ((uint32_t)buf[5] << 16) | ((uint32_t)buf[4] << 8) | buf[3];
        raw.temperature_raw = bmp581_sign_extend_24(temp_raw24);
        raw.pressure_raw = press_raw24;
    }

    *out_reading = bmp581_raw_to_reading(raw, &handle->convert, &handle->filter_state,
                                          timestamp_us, i2c_ok);
    return i2c_ok ? ESP_OK : err;
}

esp_err_t bmp581_deinit(bmp581_handle_t handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = i2c_master_bus_rm_device(handle->i2c_dev);
    free(handle);
    return err;
}
