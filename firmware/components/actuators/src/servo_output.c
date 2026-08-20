// ServoOutput implementation. All hardware access here is a single LEDC duty-cycle update in
// task context - nothing in this file runs from an ISR (same reasoning as pwm_esc_output.c).

#include "servo_output.h"

#include <stdlib.h>

#include "esp_log.h"

#include "pwm_util.h"

static const char *TAG = "servo_output";

struct servo_output_dev {
    ledc_mode_t speed_mode;
    ledc_channel_t channel;
    uint32_t freq_hz;
    uint32_t duty_resolution_bits;
    servo_convert_config_t convert;
};

static esp_err_t servo_apply_pulse_us(servo_output_handle_t dev, uint32_t pulse_us)
{
    uint32_t duty = pwm_pulse_us_to_duty(pulse_us, dev->freq_hz, dev->duty_resolution_bits);
    return ledc_set_duty_and_update(dev->speed_mode, dev->channel, duty, 0);
}

esp_err_t servo_output_init(const servo_output_config_t *config,
                             servo_output_handle_t *out_handle)
{
    if (config == NULL || out_handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    struct servo_output_dev *dev = calloc(1, sizeof(*dev));
    if (dev == NULL) {
        return ESP_ERR_NO_MEM;
    }
    dev->speed_mode = config->ledc_speed_mode;
    dev->channel = config->ledc_channel;
    dev->freq_hz = config->freq_hz;
    dev->duty_resolution_bits = config->duty_resolution_bits;
    dev->convert = config->convert;

    ledc_timer_config_t timer_cfg = {
        .speed_mode = config->ledc_speed_mode,
        .duty_resolution = (ledc_timer_bit_t)config->duty_resolution_bits,
        .timer_num = config->ledc_timer,
        .freq_hz = config->freq_hz,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    esp_err_t err = ledc_timer_config(&timer_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config failed: %s", esp_err_to_name(err));
        free(dev);
        return err;
    }

    // Channel starts at the configured neutral duty, never an undefined one - see this function's
    // header comment.
    uint32_t neutral_pulse_us =
        servo_angle_to_pulse_us(config->convert.neutral_angle_rad, &config->convert);
    uint32_t neutral_duty =
        pwm_pulse_us_to_duty(neutral_pulse_us, config->freq_hz, config->duty_resolution_bits);
    ledc_channel_config_t channel_cfg = {
        .gpio_num = config->gpio,
        .speed_mode = config->ledc_speed_mode,
        .channel = config->ledc_channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = config->ledc_timer,
        .duty = neutral_duty,
        .hpoint = 0,
    };
    err = ledc_channel_config(&channel_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_channel_config failed: %s", esp_err_to_name(err));
        free(dev);
        return err;
    }

    *out_handle = dev;
    return ESP_OK;
}

esp_err_t servo_output_set_neutral(servo_output_handle_t handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint32_t pulse_us =
        servo_angle_to_pulse_us(handle->convert.neutral_angle_rad, &handle->convert);
    return servo_apply_pulse_us(handle, pulse_us);
}

esp_err_t servo_output_write(servo_output_handle_t handle, float angle_rad)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    bool was_clamped = false;
    float clamped = servo_clamp_angle(angle_rad, &handle->convert, &was_clamped);
    if (was_clamped) {
        ESP_LOGW(TAG, "angle command %.4f rad out of range, clamped to %.4f rad",
                 (double)angle_rad, (double)clamped);
    }

    uint32_t pulse_us = servo_angle_to_pulse_us(clamped, &handle->convert);
    return servo_apply_pulse_us(handle, pulse_us);
}

esp_err_t servo_output_deinit(servo_output_handle_t handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = ledc_stop(handle->speed_mode, handle->channel, 0);
    free(handle);
    return err;
}
