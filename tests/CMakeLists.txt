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
#
# Milestone 14 (firmware/components/radio/) follows the sensors/actuators pattern again:
# radio_packet.c (wire-format serialize/parse, sequence staleness/reordering, packet-loss
# tracking) has zero ESP-IDF dependency by design, so radio_packet_test.c compiles it directly the
# same way mpu6050_convert_test.c does. The actual ESP-NOW Wi-Fi calls
# (firmware/components/radio/src/esp_now_radio.c) are hardware-dependent and not host-testable --
# see docs/radio.md. Milestone 15 adds crsf_frame.c (CRC8, frame sync, RC_CHANNELS_PACKED
# decoding, channel mapping, loss estimate) the same way -- crsf_frame_test.c compiles it directly.
# The actual UART driver calls (firmware/components/radio/src/crsf_radio.c) are likewise
# hardware-dependent and not host-testable.
#
# Milestone 16 adds two new components, both following this same pattern: safety/ (flight_mode.c,
# arming.c, failsafe.c, actuator_command_check.c -- every file in the component is ESP-IDF-free by
# design, see firmware/components/safety/CMakeLists.txt) and power/ (battery_convert.c is the pure
# half of battery monitoring; battery.c, the ESP-IDF esp_adc oneshot driver I/O, is not
# host-testable, same split as every other sensor driver).

cmake_minimum_required(VERSION 3.16)
project(bicopter_fc_tests C CXX)

enable_testing()

set(SENSORS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../firmware/components/sensors)
set(ACTUATORS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../firmware/components/actuators)
set(RADIO_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../firmware/components/radio)
set(SAFETY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../firmware/components/safety)
set(POWER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../firmware/components/power)
set(BENCH_TEST_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../firmware/components/bench_test)
set(MAIN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../firmware/main)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../flight_core flight_core-build)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../simulator/physics simulator_physics-build)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../simulator/sensors simulator_sensors-build)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../simulator/sim_loop simulator_sim_loop-build)

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

add_executable(radio_packet_test
    radio_packet_test.c
    ${RADIO_DIR}/src/radio_packet.c
)
target_include_directories(radio_packet_test PRIVATE ${RADIO_DIR}/include)
target_link_libraries(radio_packet_test m)

add_test(NAME radio_packet_test COMMAND radio_packet_test)

add_executable(crsf_frame_test
    crsf_frame_test.c
    ${RADIO_DIR}/src/crsf_frame.c
)
target_include_directories(crsf_frame_test PRIVATE ${RADIO_DIR}/include)
target_link_libraries(crsf_frame_test m)

add_test(NAME crsf_frame_test COMMAND crsf_frame_test)

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

# control_allocator_test links bicopter_physics (not just flight_core) because its round-trip
# checks feed the allocator's output through Milestone 9's real computeStateDerivative() -- see
# control_allocator_test.cpp's file header. bicopter_physics already links flight_core, so this
# also gives it flight_core/control/'s ControlAllocator via bicopter_physics's public include
# dirs plus flight_core's own.
add_executable(control_allocator_test control_allocator_test.cpp)
target_link_libraries(control_allocator_test bicopter_physics flight_core)

add_test(NAME control_allocator_test COMMAND control_allocator_test)

# sim_loop_test links bicopter_sim_loop (which itself already links bicopter_physics,
# bicopter_sim_sensors, and flight_core) for Milestone 13's closed-loop convergence tests -- see
# ../docs/simulation.md and simulator/sim_loop/include/sim_loop.h.
add_executable(sim_loop_test sim_loop_test.cpp)
target_link_libraries(sim_loop_test bicopter_sim_loop)

add_test(NAME sim_loop_test COMMAND sim_loop_test)

# Milestone 16: firmware/components/safety/ (flight-mode state machine, arming preconditions,
# failsafe detection/response, defense-in-depth actuator-command validation) and
# firmware/components/power/'s pure battery conversion math -- see docs/safety.md.
add_executable(flight_mode_test
    flight_mode_test.c
    ${SAFETY_DIR}/src/flight_mode.c
)
target_include_directories(flight_mode_test PRIVATE ${SAFETY_DIR}/include)

add_test(NAME flight_mode_test COMMAND flight_mode_test)

add_executable(arming_test
    arming_test.c
    ${SAFETY_DIR}/src/arming.c
)
target_include_directories(arming_test PRIVATE ${SAFETY_DIR}/include)

add_test(NAME arming_test COMMAND arming_test)

add_executable(failsafe_test
    failsafe_test.c
    ${SAFETY_DIR}/src/failsafe.c
)
target_include_directories(failsafe_test PRIVATE ${SAFETY_DIR}/include)

add_test(NAME failsafe_test COMMAND failsafe_test)

add_executable(actuator_command_check_test
    actuator_command_check_test.c
    ${SAFETY_DIR}/src/actuator_command_check.c
)
target_include_directories(actuator_command_check_test PRIVATE ${SAFETY_DIR}/include)
target_link_libraries(actuator_command_check_test m)

add_test(NAME actuator_command_check_test COMMAND actuator_command_check_test)

add_executable(battery_convert_test
    battery_convert_test.c
    ${POWER_DIR}/src/battery_convert.c
)
target_include_directories(battery_convert_test PRIVATE ${POWER_DIR}/include)

add_test(NAME battery_convert_test COMMAND battery_convert_test)

# Milestone 18 (see ../docs/bench_test.md): bench_test_command.c (the pure line parser behind the
# bench-test console) follows the same host-testable-pure-logic split as everything above -
# firmware/components/bench_test/src/bench_test_console.c (the ESP-IDF UART driver I/O half) is
# not host-testable, same reasoning as radio/crsf's own driver files.
add_executable(bench_test_command_test
    bench_test_command_test.c
    ${BENCH_TEST_DIR}/src/bench_test_command.c
)
target_include_directories(bench_test_command_test PRIVATE ${BENCH_TEST_DIR}/include)

add_test(NAME bench_test_command_test COMMAND bench_test_command_test)

# The BENCH_TEST motor-disable compile-time gate (docs/bench_test.md): pwm_esc_bench_test.h
# translates CONFIG_BICOPTER_BENCH_TEST_MOTORS_DISABLED into an always-defined macro. Built twice
# from the same source, once simulating every normal build (macro undefined) and once simulating a
# real BENCH_TEST build (-DCONFIG_BICOPTER_BENCH_TEST_MOTORS_DISABLED=1) - see
# pwm_esc_bench_test_gate_test.c's file header for exactly what this does and doesn't prove.
add_executable(pwm_esc_bench_test_gate_test
    pwm_esc_bench_test_gate_test.c
)
target_include_directories(pwm_esc_bench_test_gate_test PRIVATE ${ACTUATORS_DIR}/include)

add_test(NAME pwm_esc_bench_test_gate_test COMMAND pwm_esc_bench_test_gate_test)

add_executable(pwm_esc_bench_test_gate_disabled_test
    pwm_esc_bench_test_gate_test.c
)
target_include_directories(pwm_esc_bench_test_gate_disabled_test PRIVATE ${ACTUATORS_DIR}/include)
target_compile_definitions(pwm_esc_bench_test_gate_disabled_test PRIVATE
    CONFIG_BICOPTER_BENCH_TEST_MOTORS_DISABLED=1
    PWM_ESC_BENCH_TEST_GATE_EXPECT_DISABLED=1
)

add_test(NAME pwm_esc_bench_test_gate_disabled_test COMMAND pwm_esc_bench_test_gate_disabled_test)

# Follow-up task (not one of the 18 numbered milestones -- see AGENTS.md/docs/visualization.md):
# simulator/visualization/'s coordinate_convert.cpp (NED <-> VGL render-space conversion) is
# glm/VGL-free by design, so it compiles directly here the same way mpu6050_convert.c does -- this
# keeps VGL scoped entirely to simulator/visualization/'s own CMakeLists.txt (FetchContent), never
# leaking into this tests/ build.
set(VISUALIZATION_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../simulator/visualization)

add_executable(coordinate_convert_test
    coordinate_convert_test.cpp
    ${VISUALIZATION_DIR}/src/coordinate_convert.cpp
)
target_include_directories(coordinate_convert_test PRIVATE ${VISUALIZATION_DIR}/include)
target_link_libraries(coordinate_convert_test flight_core)

add_test(NAME coordinate_convert_test COMMAND coordinate_convert_test)
