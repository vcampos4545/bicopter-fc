# bicopter_sim_loop: the closed-loop wiring (Milestone 13) tying together the simulated IMU
# (../sensors/), Milestone 9's rigid-body dynamics (../physics/), and flight_core's estimator +
# attitude/rate control + allocation stack into one steppable loop -- see include/sim_loop.h and
# ../../docs/simulation.md. Standalone plain-CMake static library, same guarded-subdirectory
# pattern as ../physics/CMakeLists.txt and ../sensors/CMakeLists.txt, so it configures whether
# pulled in from tests/ or the top-level simulator/ project.

cmake_minimum_required(VERSION 3.16)
project(bicopter_sim_loop CXX)

if(NOT TARGET bicopter_physics)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../physics ${CMAKE_CURRENT_BINARY_DIR}/physics-build)
endif()
if(NOT TARGET bicopter_sim_sensors)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../sensors ${CMAKE_CURRENT_BINARY_DIR}/sensors-build)
endif()

add_library(bicopter_sim_loop STATIC
    src/sim_loop.cpp
)
target_include_directories(bicopter_sim_loop PUBLIC include)
target_link_libraries(bicopter_sim_loop PUBLIC bicopter_physics bicopter_sim_sensors)
target_compile_features(bicopter_sim_loop PUBLIC cxx_std_17)
