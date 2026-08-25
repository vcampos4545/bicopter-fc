# bicopter_visualization + bicopter_sim_viz: a real graphical visualization of the closed-loop
# simulator, on top of the captain's VGL rendering library
# (https://github.com/vcampos4545/VGL), pulled in via CMake FetchContent per VGL's own README's
# recommended approach. This is a follow-up task, not one of the 18 numbered milestones -- see
# AGENTS.md/TODO.md -- and ../../docs/visualization.md for the full writeup (coordinate
# convention, what's rendered, verification).
#
# VGL is scoped to THIS directory's CMakeLists.txt only: flight_core/, firmware/, and tests/ never
# add_subdirectory this directory and never link the `vgl` target -- ../../tests/CMakeLists.txt
# instead compiles coordinate_convert.cpp (the glm/VGL-free half) directly, the same way it
# compiles other components' pure logic (AGENTS.md's driver-testing convention).
#
# Requires VGL's own macOS/Linux dependencies -- `brew install glfw glew glm` on macOS,
# `apt install libglfw3-dev libglew-dev libglm-dev` on Ubuntu -- plus network access for
# FetchContent's git clone. ../CMakeLists.txt gates add_subdirectory(visualization) behind the
# BICOPTER_SIM_BUILD_VISUALIZATION option (default ON) so a machine without those libraries/
# network access can still configure/build the rest of simulator/ with that option turned OFF.

cmake_minimum_required(VERSION 3.16)
project(bicopter_sim_visualization CXX)

if(NOT TARGET bicopter_sim_loop)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../sim_loop ${CMAKE_CURRENT_BINARY_DIR}/sim_loop-build)
endif()

include(FetchContent)
FetchContent_Declare(
    vgl
    GIT_REPOSITORY https://github.com/vcampos4545/VGL.git
    GIT_TAG main
)
FetchContent_MakeAvailable(vgl)

add_library(bicopter_visualization STATIC
    src/coordinate_convert.cpp
    src/scene_renderer.cpp
)
target_include_directories(bicopter_visualization PUBLIC include)
target_link_libraries(bicopter_visualization PUBLIC bicopter_sim_loop vgl)
target_compile_features(bicopter_visualization PUBLIC cxx_std_17)

add_executable(bicopter_sim_viz
    src/main_viz.cpp
)
target_link_libraries(bicopter_sim_viz PRIVATE bicopter_visualization)
target_compile_features(bicopter_sim_viz PRIVATE cxx_std_17)
