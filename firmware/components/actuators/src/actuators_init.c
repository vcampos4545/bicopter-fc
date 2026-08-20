#include "actuators_init.h"

#include <string.h>

#include "esp_log.h"

static const char *TAG = "actuators_init";

static void deinit_unit(actuator_unit_t *unit)
{
    if (unit->servo != NULL) {
        servo_output_deinit(unit->servo);
        unit->servo = NULL;
    }
    if (unit->motor_handle != NULL) {
        pwm_esc_output_deinit(unit->motor_handle);
        unit->motor_handle = NULL;
    }
    // Also clear the motor_output_t vtable binding, not just motor_handle - its .ctx points at
    // the handle just freed above, and leaving it non-NULL would let a caller that ignores this
    // function's error return still call motor_output_write()/etc. on a dangling ctx.
    unit->motor.ops = NULL;
    unit->motor.ctx = NULL;
}

esp_err_t actuators_init_safe(const actuators_config_t *config, actuators_t *out_actuators)
{
    if (config == NULL || out_actuators == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(out_actuators, 0, sizeof(*out_actuators));

    for (int i = 0; i < ACTUATORS_NUM_UNITS; i++) {
        actuator_unit_t *unit = &out_actuators->units[i];
        const actuator_unit_config_t *unit_config = &config->units[i];

        esp_err_t err = pwm_esc_output_init(&unit_config->motor, &unit->motor_handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "unit %d motor init failed: %s", i, esp_err_to_name(err));
            for (int j = 0; j <= i; j++) {
                deinit_unit(&out_actuators->units[j]);
            }
            return err;
        }
        pwm_esc_output_as_motor_output(unit->motor_handle, &unit->motor);

        err = servo_output_init(&unit_config->servo, &unit->servo);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "unit %d servo init failed: %s", i, esp_err_to_name(err));
            for (int j = 0; j <= i; j++) {
                deinit_unit(&out_actuators->units[j]);
            }
            return err;
        }
    }

    ESP_LOGI(TAG, "actuators initialized: %d unit(s), motors idle, servos neutral",
             ACTUATORS_NUM_UNITS);
    return ESP_OK;
}
