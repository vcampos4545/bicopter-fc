# sensors: MPU6050 IMU driver (Milestone 3). mpu6050_convert.c/.h contain the pure
# register-to-SI/calibration/stale-detection logic and are also compiled standalone by
# ../../../tests/CMakeLists.txt for host-side unit testing, independent of ESP-IDF.

idf_component_register(SRCS "src/mpu6050.c" "src/mpu6050_convert.c"
                        INCLUDE_DIRS "include"
                        REQUIRES driver esp_timer)
