# actuators: MotorOutput/ServoOutput hardware abstractions (Milestone 5), driving 2 BLHeli ESCs
# and 2 tilt servos over LEDC-generated conventional RC PWM. PwmEscOutput implements the generic
# motor_output_t interface (motor_output.h) so a future DShot implementation is a second file
# behind the same interface, not a rewrite - see docs/hardware.md. *_convert.c/.h and pwm_util.c/
# .h contain the pure normalized-value/pulse-width/duty-count math and are also compiled
# standalone by ../../../tests/CMakeLists.txt for host-side unit testing, independent of ESP-IDF.

idf_component_register(SRCS "src/pwm_esc_output.c" "src/pwm_esc_convert.c"
                             "src/servo_output.c" "src/servo_convert.c"
                             "src/pwm_util.c" "src/actuators_init.c"
                        INCLUDE_DIRS "include"
                        REQUIRES driver)
