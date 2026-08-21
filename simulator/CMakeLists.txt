# simulator: desktop physics simulator that links flight_core unmodified
# against simulated hardware-abstraction implementations.
#
# Milestone 9 added physics/ (see physics/CMakeLists.txt) as a real, standalone static library —
# the rigid-body forward-dynamics model, see ../docs/dynamics.md. Milestone 13 adds sensors/
# (SimulatedImu, see sensors/CMakeLists.txt and ../docs/simulation.md) and sim_loop/ (the
# estimator -> attitude/rate control -> allocation -> dynamics closed loop, see
# sim_loop/CMakeLists.txt and ../docs/simulation.md), and builds main.cpp as the interactive
# bicopter_sim executable — a minimal text-trace demo of closed-loop stabilization, not a
# substitute for tests/sim_loop_test.cpp's automated convergence assertions. visualization/
# remains unimplemented — a full graphical visualization is out of scope for Milestone 13 (not
# one of the 18 numbered items). Do not add stub .cpp files for visualization/ just to make this
# configure.

cmake_minimum_required(VERSION 3.16)
project(bicopter_sim CXX)

add_subdirectory(physics)
add_subdirectory(sensors)
add_subdirectory(sim_loop)

add_executable(bicopter_sim
    main.cpp
)
target_link_libraries(bicopter_sim PRIVATE bicopter_sim_loop)
target_compile_features(bicopter_sim PRIVATE cxx_std_17)
