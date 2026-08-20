// Conventional-RC-PWM MotorOutput implementation, driving one BLHeli ESC over the ESP32's LEDC
// peripheral. Public API deliberately matches the project's `MotorOutput` HAL shape from
// docs/architecture.md: `pwm_esc_output_write()` takes a normalized throttle in [0,1], protocol
// and calibration are hidden behind this implementation. See motor_output.h for how this plugs
// into the generic interface a future DshotEscOutput would also implement, and docs/hardware.md
// for why LEDC (over MCPWM) was chosen and how ESC calibration/protocol selection works.
#ifndef BICOPTER_ACTUATORS_PWM_ESC_OUTPUT_H
#define BICOPTER_ACTUATORS_PWM_ESC_OUTPUT_H

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"

#include "motor_output.h"
#include "pwm_esc_convert.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct pwm_esc_output_dev *pwm_esc_output_handle_t;

typedef struct {
    gpio_num_t gpio;               // PWM signal GPIO to this ESC; board-level choice, TBD (see
                                     // docs/hardware.md)
    ledc_timer_t ledc_timer;       // LEDC timer providing this channel's frequency; share a timer
                                     // across channels that must run at the same freq_hz
    ledc_channel_t ledc_channel;   // LEDC channel, must be unique per simultaneously-active output
    ledc_mode_t ledc_speed_mode;   // LEDC_LOW_SPEED_MODE for both ESC and servo outputs in this
                                     // project (see docs/hardware.md); esp32 also has a high-speed
                                     // mode but nothing here needs it
    uint32_t freq_hz;               // PWM frequency, e.g. 50 for standard RC PWM; BLHeli often
                                     // tolerates higher - configurable, see docs/hardware.md
    uint32_t duty_resolution_bits; // LEDC duty resolution, e.g. 16 (LEDC_TIMER_16_BIT)
    pwm_esc_convert_config_t convert;
} pwm_esc_output_config_t;

// Configures the LEDC timer/channel for this ESC and immediately commands convert.idle_pulse_us -
// the channel never starts at an undefined or zero duty cycle (which a BLHeli ESC could read as
// "no signal" rather than a deliberate idle command), and the motor is never armed/driven to a
// non-idle output as a side effect of construction. This is what makes actuators_init_safe()
// (actuators_init.h) able to guarantee "idle before anything else touches actuators" just by
// calling this in the right order - the guarantee lives here, not in the caller's discipline.
esp_err_t pwm_esc_output_init(const pwm_esc_output_config_t *config,
                               pwm_esc_output_handle_t *out_handle);

// Populates `out_iface` with a motor_output_t bound to this handle, so callers program against
// the generic MotorOutput interface (motor_output.h) rather than this type directly - see
// motor_output.h for why.
void pwm_esc_output_as_motor_output(pwm_esc_output_handle_t handle, motor_output_t *out_iface);

// Commands convert.idle_pulse_us directly, bypassing the throttle mapping. Use this (not
// pwm_esc_output_write(handle, 0.0)) for any safe-init/disarm path: idle_pulse_us is independent
// of min_pulse_us and is meant to be trustworthy even before throttle calibration is finalized.
esp_err_t pwm_esc_output_set_idle(pwm_esc_output_handle_t handle);

// Clamps `throttle` to [min_throttle, max_throttle] (logging a warning via ESP_LOGW if the input
// was out of range or non-finite - see AGENTS.md milestone-5 brief on never accepting
// out-of-range values silently), maps it to a pulse width, and updates the LEDC duty.
esp_err_t pwm_esc_output_write(pwm_esc_output_handle_t handle, float throttle);

// Stops the LEDC channel and releases the handle. Does not command idle first - callers that want
// idle-then-release should call pwm_esc_output_set_idle() themselves before this.
esp_err_t pwm_esc_output_deinit(pwm_esc_output_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif // BICOPTER_ACTUATORS_PWM_ESC_OUTPUT_H
