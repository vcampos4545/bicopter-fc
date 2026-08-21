// Real automated tests for Milestone 13's closed loop (simulator/sim_loop/): the estimator ->
// attitude controller -> rate controller -> control allocator -> Milestone 9 dynamics cascade,
// fed by simulator/sensors/'s SimulatedImu rather than perfect ground truth. Same hand-rolled
// assert-and-report harness as tests/rate_controller_test.cpp/tests/control_allocator_test.cpp.
//
// These are genuine convergence tests, not single-step sanity checks: each case disturbs the
// vehicle's initial attitude, runs the closed loop for several simulated seconds, and asserts the
// TRUE (ground-truth) attitude error drops below a documented tolerance and then STAYS there for
// the remainder of the run (checked at many points across a tail window, not just once) -- see
// docs/simulation.md for the full convergence-criteria writeup and why truth (not the estimate)
// is the pass/fail signal.
//
// Per docs/control_allocation.md's Milestone 12 finding (re-affirmed, not contradicted, by this
// milestone): with the default VehicleParams::center_of_mass_offset_m == Vec3::Zero(), this
// vehicle's geometry has NO pitch-torque authority near hover. test_roll_disturbance_converges
// and test_roll_yaw_disturbance_converges below deliberately disturb only roll/yaw and use the
// default (zero CoM offset) vehicle for exactly that reason. test_pitch_disturbance_converges
// demonstrates pitch separately, using a nonzero center_of_mass_offset_m.z configuration, per the
// milestone brief's "configure a nonzero CoM offset ... as an additional case rather than
// replacing the default-config roll/yaw demonstrations."
#include <cmath>
#include <cstdio>
#include <vector>

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
constexpr float kDegToRad = kPi / 180.0f;

// Same vehicle fixture as tests/bicopter_dynamics_test.cpp's / tests/control_allocator_test.cpp's
// baseParams() -- reusing an already-validated configuration rather than inventing new numbers,
// per docs/simulation.md.
VehicleParams baseParams()
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

    // Nonzero drag -- unlike tests/bicopter_dynamics_test.cpp's/tests/control_allocator_test.cpp's
    // baseParams() (which zero both, to keep their exact hand-derived-closed-form checks
    // uncontaminated by an extra force term), this milestone's closed loop needs it: a combined
    // roll+yaw maneuver excites real gyroscopic cross-coupling (I*omega_dot + omega x (I*omega) =
    // tau, docs/dynamics.md) that leaks a small, steady angular rate into pitch -- an axis this
    // default (zero CoM offset) configuration has NO torque authority over
    // (docs/control_allocation.md). Without any damping that leaked pitch rate never decays
    // (nothing opposes it) and pitch grows without bound even after roll/yaw fully settle; angular
    // drag is exactly the passive damping docs/dynamics.md's "Drag" section anticipated this
    // milestone would want ("a nonzero value is there for whichever later milestone (13's
    // closed-loop stabilization, most likely) wants some velocity damping for a more well-behaved
    // simulated vehicle"). See docs/simulation.md for the full writeup and the measured effect on
    // the roll+yaw convergence case.
    p.linear_drag_coefficient_n_per_mps = 0.2f;
    p.angular_drag_coefficient_nm_per_radps = 0.05f;
    return p;
}

// The pitch-achievable variant: a nonzero vertical CoM-to-motor-plane offset, the real,
// physically-necessary condition (docs/control_allocation.md) for this vehicle topology to have
// any pitch-torque authority at all.
VehicleParams comOffsetParams()
{
    VehicleParams p = baseParams();
    p.center_of_mass_offset_m = Vec3(0.0f, 0.0f, 0.05f);
    return p;
}

// Modest, deterministic sensor noise -- enough that the estimator is genuinely fed
// noise-corrupted data (not perfect ground truth), small enough not to itself be the dominant
// source of residual attitude error. No bias: a nonzero gyro bias would leave a permanent yaw
// steady-state offset (yaw has no accel-based correction -- docs/estimation.md), which is a
// meaningful thing to characterize but not what these convergence tests are checking. See
// docs/simulation.md for the full noise-model writeup.
bicopter::SimulatedImuConfig testImuConfig(uint32_t seed)
{
    bicopter::SimulatedImuConfig cfg;
    cfg.gyro_noise_stddev_radps = Vec3(0.005f, 0.005f, 0.005f);
    cfg.accel_noise_stddev_mps2 = Vec3(0.02f, 0.02f, 0.02f);
    cfg.seed = seed;
    return cfg;
}

// kp=0 (pure gyro integration, no accelerometer correction) -- NOT a bug and NOT ignoring
// Milestone 8's estimator, but a real finding of this milestone, documented in full in
// docs/simulation.md's "Why the estimator runs with zero accelerometer correction gain during
// flight" section: for a vehicle whose thrust is generated along a BODY-FIXED axis (as this one
// is), specific_force_body == F_thrust_body / mass EXACTLY, independent of true orientation,
// whenever torque and drag are both negligible -- so accelerometer "gravity direction" correction
// is not just noisy but structurally uninformative during hover, and provably PULLS a correct
// gyro-tracked estimate back toward level regardless of the vehicle's true attitude (confirmed
// empirically: with the default kp=2.0, the true attitude error stalls partway through recovery
// while the estimated error is dragged to ~0). This is a hovering-flight-specific extension of
// docs/estimation.md's already-documented "yaw is unobservable from gravity alone" finding to
// roll/pitch as well. Accel correction remains valid and useful for its originally-tested
// scenario -- a stationary, non-thrusting vehicle (tests/complementary_filter_test.cpp) -- which
// is why the default ComplementaryFilterConfig::kp is NOT changed; this override is scoped to
// SimLoop's in-flight configuration only.
bicopter::ComplementaryFilterConfig testEstimatorConfig()
{
    bicopter::ComplementaryFilterConfig cfg;
    cfg.kp = 0.0f;
    cfg.ki = 0.0f;
    return cfg;
}

} // namespace

static int g_failures = 0;
static int g_checks = 0;

#define CHECK(cond, msg)                                                                         \
    do {                                                                                         \
        g_checks++;                                                                              \
        if (!(cond)) {                                                                           \
            g_failures++;                                                                        \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, msg);                            \
        }                                                                                         \
    } while (0)

// Runs the closed loop from a disturbed initial attitude toward level (Quaternion::Identity())
// and checks: (1) the true attitude error drops below tolerance_rad by settle_by_s, and (2) it
// stays below tolerance_rad at every logged control cycle from settle_by_s through the end of the
// run (duration_s) -- i.e. a real converge-and-hold, not a single touch. Returns the worst
// (maximum) error observed in that post-settle tail window, for diagnostic printing.
static float runConvergenceCase(const char* name, const VehicleParams& params,
                                 const Quaternion& initial_orientation, uint32_t seed,
                                 float duration_s, float settle_by_s, float tolerance_rad)
{
    SimLoopConfig config;
    config.vehicle_params = params;
    config.imu_config = testImuConfig(seed);
    config.estimator_config = testEstimatorConfig();

    SimLoop loop(config);

    RigidBodyState initial;
    initial.orientation = initial_orientation;
    // Seed the estimator with the TRUE initial orientation, not Identity() -- see
    // docs/simulation.md's "Why the estimator is seeded at the true initial attitude" section for
    // the full reasoning: this vehicle's thrust is generated along a body-fixed axis at
    // approximately constant magnitude, which makes specific_force_body == F_thrust_body/mass
    // EXACTLY independent of orientation (R^-1(R*v) == v) whenever torque and drag are both zero
    // -- a real, structural sensor-observability limit (an extension of docs/estimation.md's
    // already-documented "yaw is unobservable from gravity alone" finding, not a bug), not
    // something any amount of gain tuning fixes. This mirrors real flight-controller practice:
    // attitude is self-leveled from the accelerometer while at rest, BEFORE motors spin up to
    // hover thrust -- exactly the scenario docs/estimation.md's stationary-convergence tests
    // already validate (tests/complementary_filter_test.cpp). From that accurate seed, gyro
    // integration (not accel correction) is what keeps the estimate accurate as the closed loop
    // commands real corrective torque and the vehicle actually starts rotating.
    loop.reset(initial, initial_orientation);
    loop.setDesiredAttitude(Quaternion::Identity());

    std::vector<SimStepLog> logs;
    loop.run(duration_s, [&](const SimStepLog& log) { logs.push_back(log); });

    CHECK(!logs.empty(), "expected at least one control-cycle log");

    bool settled_by_deadline = false;
    float max_tail_error = 0.0f;
    bool any_tail_samples = false;
    for (const auto& log : logs) {
        if (log.t_s >= settle_by_s) {
            if (!settled_by_deadline) {
                settled_by_deadline = log.true_attitude_error_rad <= tolerance_rad;
            }
            any_tail_samples = true;
            max_tail_error = std::max(max_tail_error, log.true_attitude_error_rad);
        }
    }

    char msg[256];
    std::snprintf(msg, sizeof(msg), "%s: expected true attitude error <= %.4f rad by t=%.2fs",
                  name, tolerance_rad, settle_by_s);
    CHECK(settled_by_deadline, msg);

    CHECK(any_tail_samples, "expected tail-window samples to exist");
    std::snprintf(msg, sizeof(msg),
                  "%s: expected true attitude error to STAY <= %.4f rad for all t>=%.2fs "
                  "(max observed %.4f rad) -- convergence must hold, not just touch zero once",
                  name, tolerance_rad, settle_by_s, max_tail_error);
    CHECK(max_tail_error <= tolerance_rad, msg);

    std::printf("  [%s] settled_by_deadline=%s max_tail_error_deg=%.3f\n", name,
                settled_by_deadline ? "yes" : "no", max_tail_error * 180.0f / kPi);

    return max_tail_error;
}

// Roll-only disturbance, default (zero CoM offset) vehicle -- an axis this configuration has full
// authority over via differential thrust (docs/control_allocation.md's Stage 1).
static void test_roll_disturbance_converges()
{
    Quaternion initial = Quaternion::FromEulerZYX({/*roll=*/20.0f * kDegToRad, 0.0f, 0.0f});
    runConvergenceCase("roll_only", baseParams(), initial, /*seed=*/1, /*duration_s=*/4.0f,
                        /*settle_by_s=*/2.0f, /*tolerance_rad=*/5.0f * kDegToRad);
}

// Combined roll+yaw disturbance, default (zero CoM offset) vehicle -- roll via differential
// thrust, yaw via differential tilt (docs/control_allocation.md's Stage 2), simultaneously.
static void test_roll_yaw_disturbance_converges()
{
    Quaternion initial =
        Quaternion::FromEulerZYX({/*roll=*/15.0f * kDegToRad, 0.0f, /*yaw=*/20.0f * kDegToRad});
    runConvergenceCase("roll_yaw", baseParams(), initial, /*seed=*/2, /*duration_s=*/4.0f,
                        /*settle_by_s=*/2.5f, /*tolerance_rad=*/6.0f * kDegToRad);
}

// Pitch disturbance, nonzero-CoM-offset vehicle. Per docs/control_allocation.md, pitch is
// structurally unachievable near hover with the default zero CoM offset -- this case exists
// specifically to demonstrate that a vehicle configured with real pitch authority does converge,
// as an ADDITIONAL case alongside (not a replacement for) the default-config roll/yaw
// demonstrations above.
static void test_pitch_disturbance_converges_with_com_offset()
{
    Quaternion initial = Quaternion::FromEulerZYX({0.0f, /*pitch=*/15.0f * kDegToRad, 0.0f});
    runConvergenceCase("pitch_with_com_offset", comOffsetParams(), initial, /*seed=*/3,
                        /*duration_s=*/4.0f, /*settle_by_s=*/2.5f,
                        /*tolerance_rad=*/6.0f * kDegToRad);
}

int main()
{
    test_roll_disturbance_converges();
    test_roll_yaw_disturbance_converges();
    test_pitch_disturbance_converges_with_com_offset();

    std::printf("%d/%d checks passed\n", g_checks - g_failures, g_checks);
    return g_failures == 0 ? 0 : 1;
}
