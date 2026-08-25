# simulator: desktop physics simulator that links flight_core unmodified
# against simulated hardware-abstraction implementations.
#
# Milestone 9 added physics/ (see physics/CMakeLists.txt) as a real, standalone static library —
# the rigid-body forward-dynamics model, see ../docs/dynamics.md. Milestone 13 adds sensors/
# (SimulatedImu, see sensors/CMakeLists.txt and ../docs/simulation.md) and sim_loop/ (the
# estimator -> attitude/rate control -> allocation -> dynamics closed loop, see
# sim_loop/CMakeLists.txt and ../docs/simulation.md), and builds main.cpp as the interactive
# bicopter_sim executable — a minimal text-trace demo of closed-loop stabilization, not a
# substitute for tests/sim_loop_test.cpp's automated convergence assertions. A later follow-up
# task (not one of the 18 numbered milestones — see AGENTS.md) added visualization/: a real
# VGL-based graphical renderer of the same closed loop (bicopter_sim_viz), additive alongside
# bicopter_sim rather than replacing it — see visualization/CMakeLists.txt and
# ../docs/visualization.md.

cmake_minimum_required(VERSION 3.16)
project(bicopter_sim CXX)

add_subdirectory(physics)
add_subdirectory(sensors)
add_subdirectory(sim_loop)

# Off, the rest of simulator/ (including bicopter_sim) still configures/builds without VGL, its
# glfw/glew/glm dependencies, or network access (FetchContent clones VGL from GitHub) — see
# visualization/CMakeLists.txt and ../docs/visualization.md.
option(BICOPTER_SIM_BUILD_VISUALIZATION
    "Build the VGL-based graphical visualizer (bicopter_sim_viz)" ON)
if(BICOPTER_SIM_BUILD_VISUALIZATION)
    add_subdirectory(visualization)
endif()

add_executable(bicopter_sim
    main.cpp
)
target_link_libraries(bicopter_sim PRIVATE bicopter_sim_loop)
target_compile_features(bicopter_sim PRIVATE cxx_std_17)
