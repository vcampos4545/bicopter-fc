# flight_core: platform-independent flight-control/estimation library.
#
# Milestone 1 stub only — no sources exist yet. This file wires the build
# target shape (a plain CMake static library, consumable by both firmware/
# via ESP-IDF's component system and simulator/ via a normal desktop build)
# so later milestones add source files here without renegotiating the build
# structure. See ../docs/architecture.md for why this library depends only
# on the hardware-abstraction interfaces, never on ESP-IDF or desktop-only
# libraries.
#
# Deliberately not implemented yet: no estimation/control/dynamics/math
# sources exist in this repository until their respective milestones land
# (see ../TODO.md). Do not add stub .cpp files just to make this configure.

cmake_minimum_required(VERSION 3.16)
project(flight_core CXX)

# add_library(flight_core STATIC
#     # estimation/...
#     # control/...
#     # dynamics/...
#     # math/...
#     # vehicle/...
#     # safety/...
# )
# target_include_directories(flight_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
# target_compile_features(flight_core PUBLIC cxx_std_17)
