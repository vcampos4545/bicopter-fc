# bicopter_sim_sensors: simulated sensor implementations (Milestone 13). SimulatedImu synthesizes
# noisy ImuSamples from Milestone 9's RigidBodyState ground truth -- see include/simulated_imu.h
# and ../../docs/simulation.md. Standalone plain-CMake static library, same guarded-subdirectory
# pattern as ../physics/CMakeLists.txt, so it configures whether pulled in from tests/ or the
# top-level simulator/ project.

cmake_minimum_required(VERSION 3.16)
project(bicopter_sim_sensors CXX)

if(NOT TARGET bicopter_physics)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../physics ${CMAKE_CURRENT_BINARY_DIR}/physics-build)
endif()

add_library(bicopter_sim_sensors STATIC
    src/simulated_imu.cpp
)
target_include_directories(bicopter_sim_sensors PUBLIC include)
target_link_libraries(bicopter_sim_sensors PUBLIC bicopter_physics)
target_compile_features(bicopter_sim_sensors PUBLIC cxx_std_17)
