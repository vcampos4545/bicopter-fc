# Standalone host-side tests, independent of ESP-IDF and (for now) of flight_core, which is
# still an unimplemented stub (see ../flight_core/CMakeLists.txt). Compiles the sensors
# component's pure conversion/calibration/stale-detection logic directly from
# ../firmware/components/sensors/ — that logic has zero ESP-IDF dependencies by design (see
# ../firmware/components/sensors/include/mpu6050_convert.h), so it builds and runs unmodified on
# the host. This is how Milestone 3 gets real automated test coverage without any hardware or
# QEMU: firmware I/O itself can't be unit tested without silicon, but the register-scaling math
# it depends on can be, and is, here.

cmake_minimum_required(VERSION 3.16)
project(bicopter_fc_tests C)

enable_testing()

set(SENSORS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../firmware/components/sensors)

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
