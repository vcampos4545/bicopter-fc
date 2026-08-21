#include "battery.h"

#include <stdlib.h>

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"

static const char *TAG = "battery";

struct battery_dev {
    adc_oneshot_unit_handle_t adc;
    adc_cali_handle_t voltage_cali;
    adc_cali_handle_t current_cali; // NULL if current sensing not configured
    battery_config_t config;
};

static esp_err_t create_cali_scheme(adc_unit_t unit, adc_channel_t chan, adc_atten_t atten,
                                     adc_cali_handle_t *out_handle)
{
    // ESP32 (this project's pinned target - see AGENTS.md) only supports the line-fitting
    // calibration scheme; adc_cali_create_scheme_curve_fitting() would fail
    // ESP_ERR_NOT_SUPPORTED here. See battery.h's file header.
    adc_cali_line_fitting_config_t cali_cfg = {
        .unit_id = unit,
        .atten = atten,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    (void)chan; // line-fitting calibration is per-unit, not per-channel on this target
    return adc_cali_create_scheme_line_fitting(&cali_cfg, out_handle);
}

esp_err_t battery_init(const battery_config_t *config, battery_handle_t *out_handle)
{
    if (config == NULL || out_handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *out_handle = NULL;

    struct battery_dev *dev = calloc(1, sizeof(struct battery_dev));
    if (dev == NULL) {
        return ESP_ERR_NO_MEM;
    }
    dev->config = *config;

    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = config->adc_unit,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    esp_err_t err = adc_oneshot_new_unit(&unit_cfg, &dev->adc);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "adc_oneshot_new_unit failed: %s", esp_err_to_name(err));
        free(dev);
        return err;
    }

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = config->atten,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    err = adc_oneshot_config_channel(dev->adc, config->voltage_channel, &chan_cfg);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "adc_oneshot_config_channel (voltage) failed: %s", esp_err_to_name(err));
        adc_oneshot_del_unit(dev->adc);
        free(dev);
        return err;
    }

    err = create_cali_scheme(config->adc_unit, config->voltage_channel, config->atten,
                              &dev->voltage_cali);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "voltage calibration scheme init failed: %s", esp_err_to_name(err));
        adc_oneshot_del_unit(dev->adc);
        free(dev);
        return err;
    }

    if (config->convert.current_sense_scale_a_per_v != 0.0f) {
        err = adc_oneshot_config_channel(dev->adc, config->current_channel, &chan_cfg);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "adc_oneshot_config_channel (current) failed: %s",
                     esp_err_to_name(err));
            adc_cali_delete_scheme_line_fitting(dev->voltage_cali);
            adc_oneshot_del_unit(dev->adc);
            free(dev);
            return err;
        }
        err = create_cali_scheme(config->adc_unit, config->current_channel, config->atten,
                                  &dev->current_cali);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "current calibration scheme init failed: %s", esp_err_to_name(err));
            adc_cali_delete_scheme_line_fitting(dev->voltage_cali);
            adc_oneshot_del_unit(dev->adc);
            free(dev);
            return err;
        }
    }

    *out_handle = dev;
    return ESP_OK;
}

esp_err_t battery_read(battery_handle_t handle, battery_reading_t *out_reading)
{
    if (handle == NULL || out_reading == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int voltage_mv = 0;
    esp_err_t err =
        adc_oneshot_get_calibrated_result(handle->adc, handle->voltage_cali,
                                           handle->config.voltage_channel, &voltage_mv);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "battery voltage read failed: %s", esp_err_to_name(err));
        return err;
    }

    int current_mv = 0;
    if (handle->current_cali != NULL) {
        err = adc_oneshot_get_calibrated_result(handle->adc, handle->current_cali,
                                                  handle->config.current_channel, &current_mv);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "battery current read failed: %s", esp_err_to_name(err));
            return err;
        }
    }

    *out_reading = battery_convert_raw((uint32_t)voltage_mv, (uint32_t)current_mv,
                                        &handle->config.convert);
    return ESP_OK;
}

esp_err_t battery_deinit(battery_handle_t handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (handle->current_cali != NULL) {
        adc_cali_delete_scheme_line_fitting(handle->current_cali);
    }
    if (handle->voltage_cali != NULL) {
        adc_cali_delete_scheme_line_fitting(handle->voltage_cali);
    }
    esp_err_t err = adc_oneshot_del_unit(handle->adc);
    free(handle);
    return err;
}
