# Standalone host-side tests, independent of ESP-IDF. Compiles the sensors and actuators
# components' pure conversion/calibration/clamping logic directly from
# ../firmware/components/{sensors,actuators}/ — that logic has zero ESP-IDF dependencies by
# design (see ../firmware/components/sensors/include/mpu6050_convert.h and
# ../firmware/components/actuators/include/pwm_esc_convert.h), so it builds and runs unmodified
# on the host. This is how Milestones 3-5 get real automated test coverage without any hardware
# or QEMU: firmware I/O itself can't be unit tested without silicon, but the register/PWM-scaling
# math it depends on can be, and is, here. Milestone 6 (firmware/main/) is mostly FreeRTOS task/
# queue/notification/timer/mutex scaffolding that isn't meaningfully testable off-target this way
# - task_util.c's decimation check is the one piece pure enough to be worth pulling in the same way.
#
# Milestone 7 (flight_core/math/) is a real standalone plain-CMake library (see
# ../flight_core/CMakeLists.txt), so its tests link against the `flight_core` target directly via
# add_subdirectory rather than compiling its sources a second time. Milestone 9
# (simulator/physics/) follows the same pattern via ../simulator/physics/CMakeLists.txt's
# `bicopter_physics` target, which itself depends on `flight_core`.

cmake_minimum_required(VERSION 3.16)
project(bicopter_fc_tests C CXX)

enable_testing()

set(SENSORS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../firmware/components/sensors)
set(ACTUATORS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../firmware/components/actuators)
set(MAIN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../firmware/main)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../flight_core flight_core-build)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../simulator/physics simulator_physics-build)

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

add_executable(vec3_test vec3_test.cpp)
target_link_libraries(vec3_test flight_core)

add_test(NAME vec3_test COMMAND vec3_test)

add_executable(mat3_test mat3_test.cpp)
target_link_libraries(mat3_test flight_core)

add_test(NAME mat3_test COMMAND mat3_test)

add_executable(quaternion_test quaternion_test.cpp)
target_link_libraries(quaternion_test flight_core)

add_test(NAME quaternion_test COMMAND quaternion_test)

add_executable(complementary_filter_test complementary_filter_test.cpp)
target_link_libraries(complementary_filter_test flight_core)

add_test(NAME complementary_filter_test COMMAND complementary_filter_test)

add_executable(bicopter_dynamics_test bicopter_dynamics_test.cpp)
target_link_libraries(bicopter_dynamics_test bicopter_physics)

add_test(NAME bicopter_dynamics_test COMMAND bicopter_dynamics_test)

add_executable(pid_test pid_test.cpp)
target_link_libraries(pid_test flight_core)

add_test(NAME pid_test COMMAND pid_test)

add_executable(rate_controller_test rate_controller_test.cpp)
target_link_libraries(rate_controller_test flight_core)

add_test(NAME rate_controller_test COMMAND rate_controller_test)

add_executable(attitude_controller_test attitude_controller_test.cpp)
target_link_libraries(attitude_controller_test flight_core)

add_test(NAME attitude_controller_test COMMAND attitude_controller_test)
