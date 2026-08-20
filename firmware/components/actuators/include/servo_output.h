// Tilt-servo PWM output, driving one hobby-servo-style tilt actuator over the ESP32's LEDC
// peripheral. Public API deliberately matches the project's `ServoOutput` HAL shape from
// docs/architecture.md: `servo_output_write()` takes an angle in radians, servo-frame; range
// limits and center-trim are hidden behind this implementation. Unlike MotorOutput
// (motor_output.h), no protocol-swap seam is provided here - hobby/digital tilt servos are
// overwhelmingly PWM-controlled, so this milestone doesn't invent an abstraction for a protocol
// variation that doesn't exist yet (see docs/hardware.md). If a future servo part needs a
// different protocol, that's the point to add one, following pwm_esc_output.c/motor_output.h's
// pattern.
#ifndef BICOPTER_ACTUATORS_SERVO_OUTPUT_H
#define BICOPTER_ACTUATORS_SERVO_OUTPUT_H

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"

#include "servo_convert.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct servo_output_dev *servo_output_handle_t;

typedef struct {
    gpio_num_t gpio;               // PWM signal GPIO to this servo; board-level choice, TBD (see
                                     // docs/hardware.md)
    ledc_timer_t ledc_timer;       // LEDC timer providing this channel's frequency
    ledc_channel_t ledc_channel;   // LEDC channel, must be unique per simultaneously-active output
    ledc_mode_t ledc_speed_mode;   // LEDC_LOW_SPEED_MODE, same choice as pwm_esc_output.h
    uint32_t freq_hz;               // PWM frequency, typically 50Hz for hobby servos; configurable
    uint32_t duty_resolution_bits; // LEDC duty resolution, e.g. 16 (LEDC_TIMER_16_BIT)
    servo_convert_config_t convert;
} servo_output_config_t;

// Configures the LEDC timer/channel for this servo and immediately commands
// convert.neutral_angle_rad - the channel never starts at an undefined pulse width, and the servo
// is never driven to a non-neutral position as a side effect of construction. This is what makes
// actuators_init_safe() (actuators_init.h) able to guarantee "neutral before anything else
// touches actuators" just by calling this in the right order.
esp_err_t servo_output_init(const servo_output_config_t *config,
                             servo_output_handle_t *out_handle);

// Commands convert.neutral_angle_rad directly, bypassing the angle mapping's clamping/logging
// path (neutral is always in-range by construction). Use this (not
// servo_output_write(handle, 0.0)) for any safe-init path - servo-frame zero is not guaranteed to
// equal the mechanically-neutral tilt angle.
esp_err_t servo_output_set_neutral(servo_output_handle_t handle);

// Clamps `angle_rad` to [min_angle_rad, max_angle_rad] (logging a warning via ESP_LOGW if the
// input was out of range or non-finite - see AGENTS.md milestone-5 brief on never accepting
// out-of-range values silently), maps it to a pulse width, and updates the LEDC duty.
esp_err_t servo_output_write(servo_output_handle_t handle, float angle_rad);

// Stops the LEDC channel and releases the handle. Does not command neutral first - callers that
// want neutral-then-release should call servo_output_set_neutral() themselves before this.
esp_err_t servo_output_deinit(servo_output_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif // BICOPTER_ACTUATORS_SERVO_OUTPUT_H
