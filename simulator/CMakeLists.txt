# simulator: desktop physics simulator that links flight_core unmodified
# against simulated hardware-abstraction implementations.
#
# Milestone 1 stub only — no sources exist yet. This file wires the intended
# build shape (a desktop executable depending on ../flight_core) so later
# milestones (9: dynamics simulator, 13: closed-loop stabilization) add
# source files here without renegotiating the build structure. See
# ../docs/architecture.md for the simulator execution model.
#
# Deliberately not implemented yet: no physics/sensors/visualization sources
# exist in this repository until their respective milestones land (see
# ../TODO.md). Do not add stub .cpp files just to make this configure.

cmake_minimum_required(VERSION 3.16)
project(bicopter_sim CXX)

# add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../flight_core ${CMAKE_CURRENT_BINARY_DIR}/flight_core)
#
# add_executable(bicopter_sim
#     main.cpp
#     # physics/...
#     # sensors/...
#     # visualization/...
# )
# target_link_libraries(bicopter_sim PRIVATE flight_core)
# target_compile_features(bicopter_sim PRIVATE cxx_std_17)
