# firmware: ESP-IDF project for the bicopter flight controller.
#
# Milestone 1 stub only. A real ESP-IDF project requires `idf.py` scaffolding
# (this file wired to ESP-IDF's build system, main/CMakeLists.txt, an
# sdkconfig, component registration for components/{sensors,actuators,radio,
# estimation,control,safety,telemetry}/) which is Milestone 2's deliverable
# ("ESP-IDF boots") — see ../TODO.md. Left as a placeholder here so the
# directory structure matches ../docs/architecture.md ahead of that
# milestone; do not treat this as a working build.

cmake_minimum_required(VERSION 3.16)

# Milestone 2 will replace this with the real ESP-IDF project include, e.g.:
#   include($ENV{IDF_PATH}/tools/cmake/project.cmake)
#   project(bicopter_fc)
