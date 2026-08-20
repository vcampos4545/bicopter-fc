// Generic MotorOutput HAL shape (docs/architecture.md: `write(throttle: float in [0,1])` for one
// motor/ESC channel, protocol hidden behind the implementation). This is a small C "vtable"
// (ops struct + opaque ctx pointer) rather than a bare set of `pwm_esc_output_*` calls, so that a
// future digital-protocol implementation (DShot - see docs/hardware.md's ESC protocol section)
// is a second pair of files (e.g. `dshot_esc_output.c/.h`) that populates its own
// `motor_output_ops_t` and hands out a `motor_output_t` the exact same way
// `pwm_esc_output_as_motor_output()` does - any code holding a `motor_output_t` (this milestone's
// `actuators_init_safe()`, and later milestone 12's control allocation) never changes when that
// lands. `pwm_esc_output.c/.h` (conventional RC PWM) is the only implementation as of this
// milestone; this header is the seam, not the implementation.
//
// This is deliberately a local, plain-C shape (like imu.h/barometer.h in
// firmware/components/sensors/), not flight_core's eventual interface definition - flight_core is
// still an unimplemented stub as of this milestone (see AGENTS.md).
#ifndef BICOPTER_ACTUATORS_MOTOR_OUTPUT_H
#define BICOPTER_ACTUATORS_MOTOR_OUTPUT_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    // Commands a normalized throttle, 0.0 = minimum/idle (not necessarily zero PWM), 1.0 =
    // maximum. Implementations must clamp out-of-range input (see docs/hardware.md /
    // AGENTS.md milestone-5 brief) rather than passing it through unchecked.
    esp_err_t (*write)(void *ctx, float throttle);

    // Commands the implementation's configured idle output directly, bypassing throttle mapping.
    // This is what actuators_init_safe() and any future disarm path call.
    esp_err_t (*set_idle)(void *ctx);

    // Releases any hardware resources (LEDC channel/timer, etc.) held by ctx.
    esp_err_t (*deinit)(void *ctx);
} motor_output_ops_t;

// One MotorOutput instance: an ops vtable plus the opaque implementation state it operates on.
// Built by an implementation's `*_as_motor_output()` function (e.g.
// pwm_esc_output_as_motor_output()) - never constructed by hand outside that implementation.
typedef struct {
    const motor_output_ops_t *ops;
    void *ctx;
} motor_output_t;

static inline esp_err_t motor_output_write(motor_output_t *out, float throttle)
{
    return out->ops->write(out->ctx, throttle);
}

static inline esp_err_t motor_output_set_idle(motor_output_t *out)
{
    return out->ops->set_idle(out->ctx);
}

static inline esp_err_t motor_output_deinit(motor_output_t *out)
{
    return out->ops->deinit(out->ctx);
}

#ifdef __cplusplus
}
#endif

#endif // BICOPTER_ACTUATORS_MOTOR_OUTPUT_H
