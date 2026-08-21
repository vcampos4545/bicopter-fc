// The 4 actuator commands this milestone's forward-dynamics model consumes: two motor throttle
// commands (normalized [0,1], the same convention as MotorOutput::write(), Milestone 5) and two
// servo tilt angles (radians, in each motor's own tilt frame — see docs/dynamics.md's
// "Tilt-vectoring geometry" section; clamped to VehicleParams::MotorParams's tilt limits before
// use, not the caller's job).
#pragma once

namespace bicopter {

struct ActuatorCommand {
    float motor1_throttle = 0.0f;
    float motor2_throttle = 0.0f;
    float motor1_tilt_rad = 0.0f;
    float motor2_tilt_rad = 0.0f;
};

} // namespace bicopter
