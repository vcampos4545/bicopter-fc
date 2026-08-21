# flight_core: platform-independent flight-control/estimation library.
#
# A plain CMake static library, consumable both by firmware/ via ESP-IDF's component system and
# by simulator/ (milestone 9+) via a normal desktop CMake build. See ../docs/architecture.md for
# why this library depends only on the hardware-abstraction interfaces, never on ESP-IDF or
# desktop-only libraries.
#
# Milestone 7 (math/) is the first real source in this library — estimation/control/dynamics/
# vehicle/safety remain unimplemented until their respective milestones land (see ../TODO.md). Do
# not add stub .cpp files for those just to make this configure.

cmake_minimum_required(VERSION 3.16)
project(flight_core CXX)

add_library(flight_core STATIC
    math/src/vec3.cpp
    math/src/mat3.cpp
    math/src/quaternion.cpp
    # estimation/...
    # control/...
    # dynamics/...
    # vehicle/...
    # safety/...
)
target_include_directories(flight_core PUBLIC math/include)
target_compile_features(flight_core PUBLIC cxx_std_17)
