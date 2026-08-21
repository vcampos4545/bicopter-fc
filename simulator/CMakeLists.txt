# simulator: desktop physics simulator that links flight_core unmodified
# against simulated hardware-abstraction implementations.
#
# Milestone 9 adds physics/ (see physics/CMakeLists.txt) as a real, standalone static library —
# the rigid-body forward-dynamics model, see ../docs/dynamics.md. sensors/ (simulated
# Imu/Barometer HAL implementations) and visualization/ remain unimplemented; main.cpp (the
# interactive bicopter_sim executable) is deliberately not built yet either — Milestone 9's brief
# judged a smoke-test executable unnecessary given the automated test coverage under ../tests/,
# leaving that entry point for whichever future milestone (13: closed-loop stabilization, or
# earlier if a smoke test becomes useful) first needs one. Do not add stub .cpp files for
# sensors/visualization just to make this configure.

cmake_minimum_required(VERSION 3.16)
project(bicopter_sim CXX)

add_subdirectory(physics)

# add_executable(bicopter_sim
#     main.cpp
#     # sensors/...
#     # visualization/...
# )
# target_link_libraries(bicopter_sim PRIVATE bicopter_physics)
# target_compile_features(bicopter_sim PRIVATE cxx_std_17)
