# flight_core: platform-independent flight-control/estimation library.
#
# A plain CMake static library, consumable both by firmware/ via ESP-IDF's component system and
# by simulator/ (milestone 9+) via a normal desktop CMake build. See ../docs/architecture.md for
# why this library depends only on the hardware-abstraction interfaces, never on ESP-IDF or
# desktop-only libraries.
#
# Milestone 7 (math/) was the first real source in this library; Milestone 8 (estimation/) is the
# second. Milestone 9 (vehicle/) adds VehicleParams — pure configuration data, header-only, so it
# needs an include path but no new .cpp source. Milestone 10 (control/) adds the generic Pid
# building block and RateController; Milestone 11 adds AttitudeController, the outer (attitude)
# loop. dynamics/safety remain unimplemented until their respective milestones land (see
# ../TODO.md). Do not add stub .cpp files for those just to make this configure.

cmake_minimum_required(VERSION 3.16)
project(flight_core CXX)

add_library(flight_core STATIC
    math/src/vec3.cpp
    math/src/mat3.cpp
    math/src/quaternion.cpp
    estimation/src/complementary_filter.cpp
    control/src/pid.cpp
    control/src/rate_controller.cpp
    control/src/attitude_controller.cpp
    # dynamics/...
    # safety/...
)
target_include_directories(flight_core PUBLIC
    math/include
    estimation/include
    vehicle/include
    control/include
)
target_compile_features(flight_core PUBLIC cxx_std_17)
