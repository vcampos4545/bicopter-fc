// Defense-in-depth validation of a fully-allocated actuator command (Milestone 5's normalized
// MotorOutput/ServoOutput units) against configured limits, before it would reach real hardware.
// This is deliberately redundant with Milestone 5's own pwm_esc_clamp_throttle()/
// servo_clamp_angle() and Milestone 12's ControlAllocator saturation policy - the point of this
// module is to not trust every upstream layer perfectly (see docs/safety.md): a command that
// somehow reaches this layer already out of range is a bug somewhere upstream, and the safe
// response is a failsafe trigger (FAILSAFE_REASON_INVALID_ACTUATOR_COMMAND), not a silent clamp.
//
// No firmware caller feeds this from a real allocator today (FlightControlTask doesn't call
// ControlAllocator in firmware yet - see docs/safety.md's scope note), so this module is
// implemented and tested standalone, ready to gate real actuator output whenever that wiring
// lands.
#ifndef BICOPTER_SAFETY_ACTUATOR_COMMAND_CHECK_H
#define BICOPTER_SAFETY_ACTUATOR_COMMAND_CHECK_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float min_throttle; // normally 0.0
    float max_throttle; // normally 1.0
    float min_tilt_rad;
    float max_tilt_rad;
} actuator_command_limits_t;

typedef struct {
    float motor1_throttle;
    float motor2_throttle;
    float servo1_tilt_rad;
    float servo2_tilt_rad;
} actuator_command_t;

// True iff every field is finite (not NaN/Inf) and within the configured limits. A non-finite
// value fails regardless of the configured range - it can never be "in range" by definition.
bool actuator_command_is_valid(const actuator_command_t *cmd,
                                const actuator_command_limits_t *limits);

#ifdef __cplusplus
}
#endif

#endif // BICOPTER_SAFETY_ACTUATOR_COMMAND_CHECK_H
