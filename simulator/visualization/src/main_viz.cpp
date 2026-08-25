// bicopter_sim_viz: a real graphical visualization of Milestone 13's closed loop, using the
// captain's VGL rendering library (https://github.com/vcampos4545/VGL) -- see
// ../../docs/visualization.md. Runs the exact same combined roll+yaw disturbance scenario
// simulator/main.cpp's text-trace bicopter_sim demonstrates (same vehicle fixture, same
// estimator/noise configuration -- see that file's demoVehicleParams()), so this is an additive
// visualization of an already-validated closed loop, not a new/different scenario. It opens a
// real window: left-drag to orbit, right-drag to pan, scroll to zoom (VGL's OrbitalCamera).
//
// Physics/control run at SimLoop's own fixed rate (physics_dt_s / control_period_s, see
// sim_loop.h), independent of render frame rate: each rendered frame accumulates real elapsed
// wall-clock time and drains it in fixed physics_dt_s steps (a standard fixed-timestep
// accumulator, e.g. Gaffer's "Fix Your Timestep") -- so the simulation's behavior (and
// convergence time) doesn't depend on the display's refresh rate or how long a frame took to
// render.
#include <algorithm>
#include <chrono>
#include <cstdio>

#include <vgl/vgl.h>

#include "scene_renderer.h"
#include "sim_loop.h"

using bicopter::Quaternion;
using bicopter::RigidBodyState;
using bicopter::SimLoop;
using bicopter::SimLoopConfig;
using bicopter::Vec3;
using bicopter::VehicleParams;
using bicopter::visualization::SceneRenderer;

namespace {
constexpr float kPi = 3.14159265358979323846f;
constexpr float kRadToDeg = 180.0f / kPi;

// Identical to simulator/main.cpp's demoVehicleParams() -- see that file and
// tests/sim_loop_test.cpp's baseParams() for why each field is set the way it is.
VehicleParams demoVehicleParams()
{
    VehicleParams p;
    p.mass_kg = 2.0f;
    p.inertia_diag_kg_m2 = Vec3(0.02f, 0.03f, 0.04f);
    p.center_of_mass_offset_m = Vec3::Zero();

    p.motor1.arm_offset_y_m = -0.15f;
    p.motor2.arm_offset_y_m = 0.15f;
    p.motor1.spin_direction = 1.0f;
    p.motor2.spin_direction = -1.0f;
    p.motor1.thrust_coefficient_n = 20.0f;
    p.motor2.thrust_coefficient_n = 20.0f;
    p.motor1.torque_coefficient_nm_per_n = 0.0f;
    p.motor2.torque_coefficient_nm_per_n = 0.0f;
    p.motor1.max_tilt_rad = kPi / 2.0f;
    p.motor2.max_tilt_rad = kPi / 2.0f;
    p.motor1.min_tilt_rad = -kPi / 2.0f;
    p.motor2.min_tilt_rad = -kPi / 2.0f;
    p.linear_drag_coefficient_n_per_mps = 0.2f;
    p.angular_drag_coefficient_nm_per_radps = 0.05f;
    return p;
}
} // namespace

int main()
{
    SimLoopConfig config;
    config.vehicle_params = demoVehicleParams();
    config.imu_config.gyro_noise_stddev_radps = Vec3(0.005f, 0.005f, 0.005f);
    config.imu_config.accel_noise_stddev_mps2 = Vec3(0.02f, 0.02f, 0.02f);
    config.imu_config.seed = 42;
    config.estimator_config.kp = 0.0f;
    config.estimator_config.ki = 0.0f;

    SimLoop loop(config);

    RigidBodyState initial;
    initial.orientation = Quaternion::FromEulerZYX(
        {/*roll=*/15.0f / kRadToDeg, 0.0f, /*yaw=*/20.0f / kRadToDeg});
    loop.reset(initial, initial.orientation);
    loop.setDesiredAttitude(Quaternion::Identity());

    GUI gui(1280, 720, "Bicopter Sim Visualizer -- roll+yaw disturbance recovering to level");
    gui.setClearColor({0.08f, 0.09f, 0.11f});
    gui.camera.setFOV(50.0f).setClipPlanes(0.05f, 100.0f);

    OrbitalCamera orbit_camera(/*distance=*/2.0f, /*yaw=*/45.0f, /*pitch=*/25.0f,
                                /*target=*/glm::vec3(0.0f, 0.0f, 0.0f));
    orbit_camera.setMinDistance(0.3f).setMaxDistance(50.0f);

    SceneRenderer renderer(config.vehicle_params);

    glm::vec2 last_mouse_pos = gui.getMousePosition();
    auto last_time = std::chrono::steady_clock::now();
    float physics_accumulator_s = 0.0f;
    int frame = 0;

    std::printf("bicopter_sim_viz: 15deg roll + 20deg yaw disturbance recovering to level\n");
    std::printf("Left-drag: orbit  Right-drag: pan  Scroll: zoom\n");

    while (!gui.shouldClose()) {
        gui.beginFrame();

        const auto now = std::chrono::steady_clock::now();
        float frame_dt_s = std::chrono::duration<float>(now - last_time).count();
        last_time = now;
        // Clamp so a paused/breakpointed/stalled frame doesn't dump a huge, non-realtime burst of
        // physics steps into the accumulator below.
        frame_dt_s = std::min(frame_dt_s, 0.1f);
        physics_accumulator_s += frame_dt_s;

        while (physics_accumulator_s >= config.physics_dt_s) {
            loop.step();
            physics_accumulator_s -= config.physics_dt_s;
        }

        const glm::vec2 mouse_pos = gui.getMousePosition();
        orbit_camera.handleInput(gui, mouse_pos - last_mouse_pos, gui.getScrollDelta());
        orbit_camera.applyToCamera(gui.camera);
        last_mouse_pos = mouse_pos;

        renderer.render(gui, loop);

        if (frame % 120 == 0) {
            std::printf("t=%6.3fs  true_err=%6.2fdeg  est_err=%6.2fdeg\n", loop.t(),
                        loop.lastLog().true_attitude_error_rad * kRadToDeg,
                        loop.lastLog().estimated_attitude_error_rad * kRadToDeg);
        }
        ++frame;

        gui.endFrame();
    }

    return 0;
}
