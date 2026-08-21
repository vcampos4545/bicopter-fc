# bicopter_physics: the rigid-body forward-dynamics model (Milestone 9). A standalone plain-CMake
# static library, the same pattern as ../../flight_core/CMakeLists.txt, so it configures whether
# it's pulled in from tests/ (tests/CMakeLists.txt) or from the top-level simulator/ project
# (../CMakeLists.txt) without either entry point negotiating build structure with the other.
#
# Depends on flight_core (Milestone 7's math/, Milestone 9's vehicle/ config structs) but nothing
# desktop-only beyond the standard library — see ../../docs/dynamics.md.

cmake_minimum_required(VERSION 3.16)
project(bicopter_physics CXX)

# Guarded: whichever entry point (tests/CMakeLists.txt or ../CMakeLists.txt) includes this
# directory first may have already added flight_core as a subdirectory of the same build tree —
# add_subdirectory-ing the same physical directory twice is a hard CMake error.
if(NOT TARGET flight_core)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../flight_core ${CMAKE_CURRENT_BINARY_DIR}/flight_core-build)
endif()

add_library(bicopter_physics STATIC
    src/bicopter_dynamics.cpp
)
target_include_directories(bicopter_physics PUBLIC include)
target_link_libraries(bicopter_physics PUBLIC flight_core)
target_compile_features(bicopter_physics PUBLIC cxx_std_17)
