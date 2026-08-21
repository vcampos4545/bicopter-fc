// PwmEscOutput implementation. All hardware access here is a single LEDC duty-cycle update in
// task context (setting a duty cycle is not itself time-critical to the microsecond once the
// value is computed - see docs/architecture.md "Interrupt context vs. task context"); nothing in
// this file runs from an ISR.

#include "pwm_esc_output.h"

#include <stdlib.h>

#include "esp_log.h"

#include "pwm_esc_bench_test.h"
#include "pwm_util.h"

static const char *TAG = "pwm_esc";

struct pwm_esc_output_dev {
    ledc_mode_t speed_mode;
    ledc_channel_t channel;
    uint32_t freq_hz;
    uint32_t duty_resolution_bits;
    pwm_esc_convert_config_t convert;
};

// The BENCH_TEST compile-time guarantee (docs/bench_test.md) lives entirely in this function: it
// is the *only* place in this driver (in this file, or anywhere else) that ever calls
// ledc_set_duty_and_update() on an ESC channel. When PWM_ESC_BENCH_TEST_MOTORS_DISABLED is 1, the
// call is #if'd out of the compiled binary - not skipped by a runtime `if` on a value that could
// be mis-set, but genuinely absent from the object code, so no `pulse_us` argument, however
// computed, can ever reach the LEDC peripheral. See pwm_esc_output_init() below for the matching
// guard on ever attaching this channel to its GPIO in the first place.
static esp_err_t pwm_esc_apply_pulse_us(pwm_esc_output_handle_t dev, uint32_t pulse_us)
{
#if PWM_ESC_BENCH_TEST_MOTORS_DISABLED
    (void)dev;
    (void)pulse_us;
    return ESP_OK;
#else
    uint32_t duty = pwm_pulse_us_to_duty(pulse_us, dev->freq_hz, dev->duty_resolution_bits);
    return ledc_set_duty_and_update(dev->speed_mode, dev->channel, duty, 0);
#endif
}

esp_err_t pwm_esc_output_init(const pwm_esc_output_config_t *config,
                               pwm_esc_output_handle_t *out_handle)
{
    if (config == NULL || out_handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    struct pwm_esc_output_dev *dev = calloc(1, sizeof(*dev));
    if (dev == NULL) {
        return ESP_ERR_NO_MEM;
    }
    dev->speed_mode = config->ledc_speed_mode;
    dev->channel = config->ledc_channel;
    dev->freq_hz = config->freq_hz;
    dev->duty_resolution_bits = config->duty_resolution_bits;
    dev->convert = config->convert;

#if PWM_ESC_BENCH_TEST_MOTORS_DISABLED
    // BENCH_TEST build (docs/bench_test.md): this GPIO is never configured as an LEDC PWM output
    // at all - not just left at idle duty, but never attached to the LEDC peripheral in the first
    // place. Belt-and-suspenders alongside pwm_esc_apply_pulse_us()'s own guard above: even a bug
    // that somehow reached this channel's ledc_channel/ledc_speed_mode fields directly (bypassing
    // this driver's API entirely) would still find no LEDC channel claiming this GPIO.
    ESP_LOGW(TAG,
             "BENCH_TEST build (CONFIG_BICOPTER_BENCH_TEST_MOTORS_DISABLED=y): motor GPIO %d NOT "
             "configured as a PWM output - no motor-spinning signal can be produced this build. "
             "See docs/bench_test.md.",
             (int)config->gpio);
#else
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

    // Channel starts at the configured idle duty, never an undefined/zero one - see this
    // function's header comment.
    uint32_t idle_duty = pwm_pulse_us_to_duty(config->convert.idle_pulse_us, config->freq_hz,
                                               config->duty_resolution_bits);
    ledc_channel_config_t channel_cfg = {
        .gpio_num = config->gpio,
        .speed_mode = config->ledc_speed_mode,
        .channel = config->ledc_channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = config->ledc_timer,
        .duty = idle_duty,
        .hpoint = 0,
    };
    err = ledc_channel_config(&channel_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_channel_config failed: %s", esp_err_to_name(err));
        free(dev);
        return err;
    }
#endif // PWM_ESC_BENCH_TEST_MOTORS_DISABLED

    *out_handle = dev;
    return ESP_OK;
}

esp_err_t pwm_esc_output_set_idle(pwm_esc_output_handle_t handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    return pwm_esc_apply_pulse_us(handle, handle->convert.idle_pulse_us);
}

esp_err_t pwm_esc_output_write(pwm_esc_output_handle_t handle, float throttle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    bool was_clamped = false;
    float clamped = pwm_esc_clamp_throttle(throttle, &handle->convert, &was_clamped);
    if (was_clamped) {
        ESP_LOGW(TAG, "throttle command %.4f out of range, clamped to %.4f", (double)throttle,
                 (double)clamped);
    }

    uint32_t pulse_us = pwm_esc_throttle_to_pulse_us(clamped, &handle->convert);
    return pwm_esc_apply_pulse_us(handle, pulse_us);
}

esp_err_t pwm_esc_output_deinit(pwm_esc_output_handle_t handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
#if PWM_ESC_BENCH_TEST_MOTORS_DISABLED
    // No LEDC channel was ever configured for this handle (see pwm_esc_output_init()) - nothing
    // to stop.
    esp_err_t err = ESP_OK;
#else
    esp_err_t err = ledc_stop(handle->speed_mode, handle->channel, 0);
#endif
    free(handle);
    return err;
}

static esp_err_t pwm_esc_ops_write(void *ctx, float throttle)
{
    return pwm_esc_output_write((pwm_esc_output_handle_t)ctx, throttle);
}

static esp_err_t pwm_esc_ops_set_idle(void *ctx)
{
    return pwm_esc_output_set_idle((pwm_esc_output_handle_t)ctx);
}

static esp_err_t pwm_esc_ops_deinit(void *ctx)
{
    return pwm_esc_output_deinit((pwm_esc_output_handle_t)ctx);
}

static const motor_output_ops_t s_pwm_esc_motor_output_ops = {
    .write = pwm_esc_ops_write,
    .set_idle = pwm_esc_ops_set_idle,
    .deinit = pwm_esc_ops_deinit,
};

void pwm_esc_output_as_motor_output(pwm_esc_output_handle_t handle, motor_output_t *out_iface)
{
    out_iface->ops = &s_pwm_esc_motor_output_ops;
    out_iface->ctx = handle;
}
