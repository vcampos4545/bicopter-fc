# sensors: MPU6050 IMU driver (Milestone 3) and BMP581 barometer driver (Milestone 4).
# *_convert.c/.h files in each pair contain the pure register-to-SI/calibration/filtering/
# stale-detection logic and are also compiled standalone by ../../../tests/CMakeLists.txt for
# host-side unit testing, independent of ESP-IDF.

idf_component_register(SRCS "src/mpu6050.c" "src/mpu6050_convert.c"
                             "src/bmp581.c" "src/bmp581_convert.c"
                        INCLUDE_DIRS "include"
                        REQUIRES driver esp_timer)
