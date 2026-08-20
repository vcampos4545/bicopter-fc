# Standalone host-side tests, independent of ESP-IDF and (for now) of flight_core, which is
# still an unimplemented stub (see ../flight_core/CMakeLists.txt). Compiles the sensors and
# actuators components' pure conversion/calibration/clamping logic directly from
# ../firmware/components/{sensors,actuators}/ — that logic has zero ESP-IDF dependencies by
# design (see ../firmware/components/sensors/include/mpu6050_convert.h and
# ../firmware/components/actuators/include/pwm_esc_convert.h), so it builds and runs unmodified
# on the host. This is how Milestones 3-5 get real automated test coverage without any hardware
# or QEMU: firmware I/O itself can't be unit tested without silicon, but the register/PWM-scaling
# math it depends on can be, and is, here. Milestone 6 (firmware/main/) is mostly FreeRTOS task/
# queue/notification/timer/mutex scaffolding that isn't meaningfully testable off-target this way
# - task_util.c's decimation check is the one piece pure enough to be worth pulling in the same way.

cmake_minimum_required(VERSION 3.16)
project(bicopter_fc_tests C)

enable_testing()

set(SENSORS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../firmware/components/sensors)
set(ACTUATORS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../firmware/components/actuators)
set(MAIN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../firmware/main)

add_executable(mpu6050_convert_test
    mpu6050_convert_test.c
    ${SENSORS_DIR}/src/mpu6050_convert.c
)
target_include_directories(mpu6050_convert_test PRIVATE ${SENSORS_DIR}/include)
target_link_libraries(mpu6050_convert_test m)

add_test(NAME mpu6050_convert_test COMMAND mpu6050_convert_test)

add_executable(bmp581_convert_test
    bmp581_convert_test.c
    ${SENSORS_DIR}/src/bmp581_convert.c
)
target_include_directories(bmp581_convert_test PRIVATE ${SENSORS_DIR}/include)
target_link_libraries(bmp581_convert_test m)

add_test(NAME bmp581_convert_test COMMAND bmp581_convert_test)

add_executable(pwm_esc_convert_test
    pwm_esc_convert_test.c
    ${ACTUATORS_DIR}/src/pwm_esc_convert.c
    ${ACTUATORS_DIR}/src/pwm_util.c
)
target_include_directories(pwm_esc_convert_test PRIVATE ${ACTUATORS_DIR}/include)
target_link_libraries(pwm_esc_convert_test m)

add_test(NAME pwm_esc_convert_test COMMAND pwm_esc_convert_test)

add_executable(servo_convert_test
    servo_convert_test.c
    ${ACTUATORS_DIR}/src/servo_convert.c
    ${ACTUATORS_DIR}/src/pwm_util.c
)
target_include_directories(servo_convert_test PRIVATE ${ACTUATORS_DIR}/include)
target_link_libraries(servo_convert_test m)

add_test(NAME servo_convert_test COMMAND servo_convert_test)

add_executable(pwm_util_test
    pwm_util_test.c
    ${ACTUATORS_DIR}/src/pwm_util.c
)
target_include_directories(pwm_util_test PRIVATE ${ACTUATORS_DIR}/include)
target_link_libraries(pwm_util_test m)

add_test(NAME pwm_util_test COMMAND pwm_util_test)

add_executable(task_util_test
    task_util_test.c
    ${MAIN_DIR}/task_util.c
)
target_include_directories(task_util_test PRIVATE ${MAIN_DIR})

add_test(NAME task_util_test COMMAND task_util_test)
