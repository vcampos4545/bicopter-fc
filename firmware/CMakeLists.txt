# firmware: ESP-IDF project for the bicopter flight controller.
#
# Targets the plain "esp32" chip (no specific board/pinout assumed — see
# docs/hardware.md). Component registration for components/{sensors,actuators,
# radio,estimation,control,safety,telemetry}/ is added as each of those lands
# in its own milestone; only main/ is real as of Milestone 2. See
# docs/firmware.md for install/build/verify instructions.

cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(bicopter_fc)
