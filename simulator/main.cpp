// bicopter_sim: a minimal text-based demonstration of Milestone 13's closed loop -- a combined
// roll+yaw disturbance recovering to level under the default (zero center-of-mass-offset)
// vehicle configuration, tracing attitude error to stdout every few control cycles so convergence
// is directly observable, not just asserted in a test. See docs/simulation.md, which this demo's
// configuration matches (same vehicle fixture, same estimator/noise configuration, same reasoning
// for both) so results here and in tests/sim_loop_test.cpp agree. A real graphical visualization
// of this same scenario, built on VGL, lives alongside this one as `bicopter_sim_viz` -- see
// visualization/ and docs/visualization.md; this text-trace demo stays as the lightweight/
// no-GUI-dependency build path.
//
// This is a smoke-test/demo entry point, not itself part of the automated test suite --
// tests/sim_loop_test.cpp is what asserts convergence. Run with `./bicopter_sim` after building
// simulator/ (see README.md's simulator/ build instructions).
#include <cmath>
#include <cstdio>

#include "sim_loop.h"

using bicopter::Quaternion;
using bicopter::RigidBodyState;
using bicopter::SimLoop;
using bicopter::SimLoopConfig;
using bicopter::SimStepLog;
using bicopter::Vec3;
using bicopter::VehicleParams;

namespace {
constexpr float kPi = 3.14159265358979323846f;
constexpr float kRadToDeg = 180.0f / kPi;

// Same vehicle fixture as tests/sim_loop_test.cpp's baseParams() -- see that file and
// docs/simulation.md for why each field (including the nonzero drag) is set the way it is.
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
    // A little sensor noise so the demo shows a real (not perfect-ground-truth) closed loop --
    // see docs/simulation.md's "Simulated IMU model" section for these numbers' provenance.
    config.imu_config.gyro_noise_stddev_radps = Vec3(0.005f, 0.005f, 0.005f);
    config.imu_config.accel_noise_stddev_mps2 = Vec3(0.02f, 0.02f, 0.02f);
    config.imu_config.seed = 42;
    // kp=0: pure gyro integration, no accelerometer correction -- see docs/simulation.md's "Why
    // the estimator runs with zero accelerometer correction gain during flight" section for the
    // structural (not a bug, not a tuning shortcut) reason this vehicle's hovering flight regime
    // needs this.
    config.estimator_config.kp = 0.0f;
    config.estimator_config.ki = 0.0f;

    SimLoop loop(config);

    RigidBodyState initial;
    initial.orientation = Quaternion::FromEulerZYX(
        {/*roll=*/15.0f / kRadToDeg, 0.0f, /*yaw=*/20.0f / kRadToDeg});
    // Seed the estimator with the TRUE initial attitude -- see docs/simulation.md and
    // tests/sim_loop_test.cpp for why (accelerometer-based self-leveling calibration happens
    // before motors spin up to hover thrust, matching docs/estimation.md's already-validated
    // stationary-convergence scenario).
    loop.reset(initial, initial.orientation);
    loop.setDesiredAttitude(Quaternion::Identity());

    std::printf("Milestone 13 demo: 15deg roll + 20deg yaw disturbance, default (zero CoM "
                "offset) vehicle, recovering to level\n");
    std::printf("t_s     true_err_deg  est_err_deg   m1_thr  m2_thr  m1_tilt_deg  m2_tilt_deg\n");

    int cycle = 0;
    loop.run(4.0f, [&](const SimStepLog& log) {
        if (cycle % 25 == 0) {
            std::printf("%6.3f  %10.3f   %10.3f   %6.3f  %6.3f  %10.3f   %10.3f\n", log.t_s,
                        log.true_attitude_error_rad * kRadToDeg,
                        log.estimated_attitude_error_rad * kRadToDeg, log.command.motor1_throttle,
                        log.command.motor2_throttle, log.command.motor1_tilt_rad * kRadToDeg,
                        log.command.motor2_tilt_rad * kRadToDeg);
        }
        ++cycle;
    });

    std::printf("final true attitude error: %.3f deg\n",
                loop.lastLog().true_attitude_error_rad * kRadToDeg);
    return 0;
}
