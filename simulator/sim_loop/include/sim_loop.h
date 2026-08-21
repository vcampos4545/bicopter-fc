// The closed loop (Milestone 13): ties together the simulated IMU (../sensors/), Milestone 9's
// rigid-body dynamics (../physics/), and flight_core's estimator + attitude/rate control +
// allocation stack into one steppable simulation. See docs/simulation.md for the full data-flow
// description, the physics/IMU/control cadence relationship, which vehicle configuration(s) were
// demonstrated, and any gain tuning performed against this loop.
#pragma once

#include <functional>

#include "attitude_controller.h"
#include "bicopter_dynamics.h"
#include "complementary_filter.h"
#include "control_allocator.h"
#include "quaternion.h"
#include "rate_controller.h"
#include "rigid_body_state.h"
#include "simulated_imu.h"
#include "vehicle_params.h"

namespace bicopter {

struct SimLoopConfig {
    VehicleParams vehicle_params;
    SimulatedImuConfig imu_config;
    ComplementaryFilterConfig estimator_config;
    AttitudeControllerConfig attitude_controller_config;
    RateControllerConfig rate_controller_config;
    ControlAllocatorConfig allocator_config;

    // Physics integration step AND the simulated IMU's sample period -- this milestone keeps
    // these equal (one "tick" is both a physics step and a candidate IMU sample) rather than
    // sub-stepping physics between samples; see docs/simulation.md for why that's an acceptable
    // simplification. Defaults to 2 ms / 500 Hz, matching SensorTask's real target rate
    // (firmware/main/task_config.h, docs/architecture.md's FreeRTOS task table).
    float physics_dt_s = 0.002f;

    // Flight-control cascade (attitude -> rate -> allocation) update period, seconds. Internally
    // rounded to the nearest whole number of physics/IMU ticks (>=1) -- see docs/simulation.md.
    // Defaults to 4 ms / 250 Hz, matching FlightControlTask's real target rate.
    float control_period_s = 0.004f;

    // Total desired thrust commanded to ControlAllocator every control cycle, Newtons. <= 0
    // (the default) means "hover": vehicle_params.mass_kg * standard gravity, computed at
    // reset() -- this milestone has no altitude/throttle source yet (that's a later milestone),
    // so hover thrust is the only demand exercised.
    float desired_thrust_n = 0.0f;
};

// One control-cycle's worth of loggable state -- what SimLoop::run()'s optional trace callback
// receives, and what tests/docs/simulation.md use to describe convergence.
struct SimStepLog {
    float t_s = 0.0f;
    Quaternion true_orientation = Quaternion::Identity();
    Quaternion estimated_orientation = Quaternion::Identity();
    // Geodesic angle (radians) between true_orientation and the current desired attitude -- the
    // ground-truth convergence metric tests assert against.
    float true_attitude_error_rad = 0.0f;
    // Same angle, but between the estimator's orientation and the desired attitude -- included
    // for observability (how well the estimator itself is tracking), not the primary pass/fail
    // metric.
    float estimated_attitude_error_rad = 0.0f;
    AllocatedCommand command;
};

// Geodesic angle (radians, [0, pi]) between two body-to-world attitude quaternions -- the
// "attitude error" metric used throughout this milestone's tests and docs/simulation.md.
float quaternionAngleRad(const Quaternion& a, const Quaternion& b);

class SimLoop {
public:
    explicit SimLoop(const SimLoopConfig& config = {});

    // Resets true state, the estimator (seeded with initial_attitude_estimate), the simulated
    // IMU's sample schedule and noise stream, sim time, and the actuator command (held at hover
    // trim -- desired thrust, zero torque -- until the first control cycle runs).
    void reset(const RigidBodyState& initial_state,
               const Quaternion& initial_attitude_estimate = Quaternion::Identity());

    void setDesiredAttitude(const Quaternion& desired) { desired_attitude_ = desired; }
    const Quaternion& desiredAttitude() const { return desired_attitude_; }

    // Advances by exactly one physics_dt_s tick: computes the current specific force from the
    // true state and the currently-applied actuator command, samples the IMU if due, feeds the
    // estimator, runs the attitude -> rate -> allocation cascade if a control cycle is due
    // (holding the previous actuator command otherwise -- zero-order hold), then integrates
    // Milestone 9's dynamics forward by physics_dt_s using that command.
    //
    // Returns the most recent SimStepLog -- i.e. the last control cycle's result, whether or not
    // this particular tick ran one, so callers stepping at physics rate always read a sensible,
    // up-to-date log.
    const SimStepLog& step();

    // Calls step() repeatedly for duration_s of simulated time. If trace is set, it's invoked
    // once per control cycle (not every physics tick) with that cycle's SimStepLog.
    void run(float duration_s, const std::function<void(const SimStepLog&)>& trace = nullptr);

    const RigidBodyState& trueState() const { return true_state_; }
    const AttitudeEstimate& estimate() const { return last_estimate_; }
    float t() const { return t_; }
    const SimStepLog& lastLog() const { return last_log_; }

private:
    SimLoopConfig config_;

    ComplementaryFilterEstimator estimator_;
    AttitudeController attitude_controller_;
    RateController rate_controller_;
    ControlAllocator allocator_;
    SimulatedImu imu_;

    Quaternion desired_attitude_ = Quaternion::Identity();
    RigidBodyState true_state_;
    AttitudeEstimate last_estimate_;
    ActuatorCommand last_command_;
    SimStepLog last_log_;

    float t_ = 0.0f;
    unsigned control_decimation_ = 1;
    unsigned ticks_since_control_ = 0;
    float desired_thrust_n_ = 0.0f;

    void runControlCycle();
};

} // namespace bicopter
