# firmware: ESP-IDF project for the bicopter flight controller.
#
# Targets the plain "esp32" chip (no specific board/pinout assumed — see
# docs/hardware.md). Component registration for components/{estimation,
# control,telemetry}/ is added as each of those lands in its own milestone;
# main/, components/sensors/ (MPU6050 + BMP581 drivers), components/actuators/
# (ESC/servo PWM output), components/radio/ (ESP-NOW + CRSF), components/safety/
# (flight-mode/arming/failsafe logic), and components/power/ (battery ADC
# monitoring) are real as of Milestones 3-6 and 14-16.
# See docs/firmware.md for install/build/verify instructions.

cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(bicopter_fc)
