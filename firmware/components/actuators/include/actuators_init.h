// Project-wide safe-init entry point for actuators, per the top-level design brief's hard
// requirement (see AGENTS.md milestone-5 scope): on startup, motors must not be armed/active and
// servos must move to a safe neutral position before anything else touches actuators. This is
// implemented as an explicit, obvious code path rather than a convention callers have to
// remember: `actuators_init_safe()` is the *only* function in this component that produces
// working motor_output_t/servo_output_handle_t handles, and by the time it returns ESP_OK, both
// units are already idle/neutral (each unit's own init() commands idle/neutral as a side effect
// of construction - see pwm_esc_output_init()/servo_output_init() - so this function has no
// window where a handle exists but hasn't been idled/neutraled yet). Not wired into `main/` yet -
// that's milestone 6's task-architecture job (see AGENTS.md "What's real vs. stub right now").
#ifndef BICOPTER_ACTUATORS_INIT_H
#define BICOPTER_ACTUATORS_INIT_H

#include "esp_err.h"

#include "motor_output.h"
#include "pwm_esc_output.h"
#include "servo_output.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ACTUATORS_NUM_UNITS 2 // 2 motor+servo tilt-rotor units (see docs/hardware.md); the
                               // vehicle *topology* is fixed per AGENTS.md, unlike part config

// Config for one tilt-rotor unit: one ESC-driven motor + one tilt servo.
typedef struct {
    pwm_esc_output_config_t motor;
    servo_output_config_t servo;
} actuator_unit_config_t;

typedef struct {
    actuator_unit_config_t units[ACTUATORS_NUM_UNITS];
} actuators_config_t;

// One initialized tilt-rotor unit. `motor` is the generic MotorOutput interface bound to
// `motor_handle` - control-allocation code (milestone 12+) should hold onto `motor`, not
// `motor_handle`, so it stays agnostic to PWM vs. a future DShot implementation.
typedef struct {
    pwm_esc_output_handle_t motor_handle;
    motor_output_t motor;
    servo_output_handle_t servo;
} actuator_unit_t;

typedef struct {
    actuator_unit_t units[ACTUATORS_NUM_UNITS];
} actuators_t;

// Initializes both tilt-rotor units in order, leaving every motor at its configured idle pulse
// and every servo at its configured neutral angle before returning ESP_OK. On any unit's init
// failure, every handle already created in `out_actuators` (including the failing unit's motor,
// if its servo was the one that failed) is deinitialized before returning the error - callers
// never receive a partially-populated `actuators_t` to accidentally use.
esp_err_t actuators_init_safe(const actuators_config_t *config, actuators_t *out_actuators);

#ifdef __cplusplus
}
#endif

#endif // BICOPTER_ACTUATORS_INIT_H
