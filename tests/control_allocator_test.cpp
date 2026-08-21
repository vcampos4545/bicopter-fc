// Real automated tests for flight_core/control/control_allocator.{h,cpp}. Same hand-rolled
// assert-and-report harness as tests/rate_controller_test.cpp/tests/attitude_controller_test.cpp.
//
// The most important tests here are the round-trip checks: they take the allocator's output,
// build a simulator::ActuatorCommand from it, and feed it through Milestone 9's REAL
// computeStateDerivative() (not a reimplementation) to confirm the resulting thrust/torque
// approximately reproduces what was requested -- i.e. that this milestone's derived inverse
// actually is an inverse of Milestone 9's forward model, not just plausible-looking code. See
// docs/control_allocation.md for the full derivation these tests check against.
#include <cmath>
#include <cstdio>

#include "bicopter_dynamics.h" // simulator/physics/ -- for the real forward-model round-trip check
#include "control_allocator.h"

using bicopter::ActuatorCommand;
using bicopter::AllocatedCommand;
using bicopter::ControlAllocator;
using bicopter::ControlAllocatorConfig;
using bicopter::RigidBodyState;
using bicopter::StateDerivative;
using bicopter::Vec3;
using bicopter::VehicleParams;

namespace {
constexpr float kPi = 3.14159265358979323846f;
constexpr float kGravityMps2 = 9.80665f; // must match bicopter_dynamics.cpp's internal constant
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

#define CHECK_NEAR(a, b, tol, msg) CHECK(std::fabs((a) - (b)) <= (tol), msg)

// Same vehicle used by tests/bicopter_dynamics_test.cpp's baseParams(), so numbers can be
// cross-checked by hand against that file's derivations too. center_of_mass_offset_m stays at
// its default Vec3::Zero() here -- see comOffsetParams() below for the pitch-achievable variant.
static VehicleParams baseParams()
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

    p.linear_drag_coefficient_n_per_mps = 0.0f;
    p.angular_drag_coefficient_nm_per_radps = 0.0f;
    return p;
}

// A nonzero vertical CoM-to-motor-plane offset -- the real, physically-necessary condition
// (see docs/control_allocation.md's "Controllability near hover" section) for this vehicle
// topology to have any pitch-torque authority at all.
static VehicleParams comOffsetParams()
{
    VehicleParams p = baseParams();
    p.center_of_mass_offset_m = Vec3(0.0f, 0.0f, 0.05f);
    return p;
}

static float hoverThrustN(const VehicleParams& params)
{
    return params.mass_kg * kGravityMps2;
}

// Feeds an AllocatedCommand through Milestone 9's real forward model (state at rest, identity
// orientation, zero angular velocity, so world frame == body frame and the gyroscopic term
// vanishes) and returns the resulting (thrust-along-minus-Z, torque) pair, the ground truth this
// allocator's derivation is checked against.
struct AchievedEffect {
    float thrust_n = 0.0f; // achieved total thrust magnitude along body -Z
    Vec3 torque_nm = Vec3::Zero();
};

static AchievedEffect feedThroughForwardModel(const VehicleParams& params,
                                               const AllocatedCommand& cmd)
{
    ActuatorCommand command;
    command.motor1_throttle = cmd.motor1_throttle;
    command.motor2_throttle = cmd.motor2_throttle;
    command.motor1_tilt_rad = cmd.motor1_tilt_rad;
    command.motor2_tilt_rad = cmd.motor2_tilt_rad;

    RigidBodyState state; // rest, identity orientation, zero angular velocity
    const StateDerivative deriv = bicopter::computeStateDerivative(state, params, command);

    // net_force_world = gravity + thrust (orientation is identity, so world==body here) + drag(0).
    // Solve back out the thrust-only contribution: thrust_force_body = accel*mass - gravity.
    const Vec3 gravity_force(0.0f, 0.0f, params.mass_kg * kGravityMps2);
    const Vec3 thrust_force_body = deriv.acceleration_mps2 * params.mass_kg - gravity_force;

    AchievedEffect out;
    out.thrust_n = -thrust_force_body.z;
    out.torque_nm = Vec3(deriv.angular_acceleration_radps2.x * params.inertia_diag_kg_m2.x,
                          deriv.angular_acceleration_radps2.y * params.inertia_diag_kg_m2.y,
                          deriv.angular_acceleration_radps2.z * params.inertia_diag_kg_m2.z);
    return out;
}

static bool inRange(float v, float lo, float hi)
{
    return std::isfinite(v) && v >= lo - 1e-4f && v <= hi + 1e-4f;
}

static void checkCommandInBounds(const VehicleParams& params, const ControlAllocatorConfig& config,
                                  const AllocatedCommand& cmd, const char* label)
{
    char msg[256];
    std::snprintf(msg, sizeof(msg), "%s: motor1_throttle finite and in [min,max]", label);
    CHECK(inRange(cmd.motor1_throttle, config.throttle_min, config.throttle_max), msg);
    std::snprintf(msg, sizeof(msg), "%s: motor2_throttle finite and in [min,max]", label);
    CHECK(inRange(cmd.motor2_throttle, config.throttle_min, config.throttle_max), msg);
    std::snprintf(msg, sizeof(msg), "%s: motor1_tilt_rad finite and in limits", label);
    CHECK(inRange(cmd.motor1_tilt_rad, params.motor1.min_tilt_rad, params.motor1.max_tilt_rad),
          msg);
    std::snprintf(msg, sizeof(msg), "%s: motor2_tilt_rad finite and in limits", label);
    CHECK(inRange(cmd.motor2_tilt_rad, params.motor2.min_tilt_rad, params.motor2.max_tilt_rad),
          msg);
}

static void test_hover_symmetric_zero_tilt()
{
    const VehicleParams params = baseParams();
    const ControlAllocator allocator(params);

    const AllocatedCommand cmd = allocator.allocate(hoverThrustN(params), Vec3::Zero());

    CHECK(!cmd.saturated, "hover: pure hover command is well within the vehicle's envelope");
    CHECK_NEAR(cmd.motor1_throttle, cmd.motor2_throttle, 1e-5f,
               "hover: zero torque demand produces symmetric motor throttles");
    CHECK_NEAR(cmd.motor1_tilt_rad, 0.0f, 1e-5f, "hover: zero tilt on motor1");
    CHECK_NEAR(cmd.motor2_tilt_rad, 0.0f, 1e-5f, "hover: zero tilt on motor2");

    const AchievedEffect achieved = feedThroughForwardModel(params, cmd);
    CHECK_NEAR(achieved.thrust_n, hoverThrustN(params), 1e-3f,
               "hover: forward model reproduces the requested total thrust");
    CHECK(achieved.torque_nm.length() < 1e-3f,
          "hover: forward model reproduces ~zero net torque");
}

static void test_pure_roll_produces_differential_thrust()
{
    const VehicleParams params = baseParams();
    const ControlAllocator allocator(params);
    const float T = hoverThrustN(params);
    const Vec3 desired_torque(0.5f, 0.0f, 0.0f);

    const AllocatedCommand cmd = allocator.allocate(T, desired_torque);
    CHECK(!cmd.saturated, "pure roll: modest roll demand stays within the envelope");
    CHECK(cmd.motor1_throttle != cmd.motor2_throttle,
          "pure roll: nonzero roll torque produces differential motor throttle");
    CHECK_NEAR(cmd.motor1_tilt_rad, 0.0f, 1e-4f,
               "pure roll: roll authority comes from thrust, not tilt -- tilt stays ~zero");
    CHECK_NEAR(cmd.motor2_tilt_rad, 0.0f, 1e-4f,
               "pure roll: roll authority comes from thrust, not tilt -- tilt stays ~zero");

    const AchievedEffect achieved = feedThroughForwardModel(params, cmd);
    CHECK_NEAR(achieved.thrust_n, T, 1e-3f, "pure roll: total thrust still matches request");
    CHECK_NEAR(achieved.torque_nm.x, desired_torque.x, 5e-3f,
               "pure roll: forward model reproduces the requested roll torque");
    CHECK_NEAR(achieved.torque_nm.y, 0.0f, 1e-3f, "pure roll: no incidental pitch torque");
    CHECK_NEAR(achieved.torque_nm.z, 0.0f, 1e-3f, "pure roll: no incidental yaw torque");
}

static void test_pure_yaw_produces_differential_tilt()
{
    const VehicleParams params = baseParams(); // center_of_mass_offset_m.z == 0 (default)
    const ControlAllocator allocator(params);
    const float T = hoverThrustN(params);
    const Vec3 desired_torque(0.0f, 0.0f, 0.5f);

    const AllocatedCommand cmd = allocator.allocate(T, desired_torque);
    CHECK(!cmd.saturated, "pure yaw: modest yaw demand stays within the envelope");
    CHECK_NEAR(cmd.motor1_throttle, cmd.motor2_throttle, 1e-4f,
               "pure yaw: yaw authority comes from tilt, not thrust -- throttles stay symmetric");
    CHECK(cmd.motor1_tilt_rad * cmd.motor2_tilt_rad < 0.0f,
          "pure yaw: differential (opposite-sign) tilt is this geometry's yaw mechanism");
    CHECK_NEAR(cmd.motor1_tilt_rad, -cmd.motor2_tilt_rad, 1e-3f,
               "pure yaw: with symmetric arms and equal thrust, the differential tilt is "
               "symmetric too");

    const AchievedEffect achieved = feedThroughForwardModel(params, cmd);
    // Total thrust here is only approximately preserved: the linearization assumes cos(tilt)~=1,
    // so a ~10-degree differential tilt (needed to produce this yaw torque) costs a small,
    // expected amount of vertical thrust -- see docs/control_allocation.md's "Accuracy and its
    // limits" section. 3% of T comfortably covers that without masking a real regression.
    CHECK_NEAR(achieved.thrust_n, T, 0.03f * T, "pure yaw: total thrust still close to request");
    CHECK_NEAR(achieved.torque_nm.z, desired_torque.z, 1e-2f,
               "pure yaw: forward model reproduces the requested yaw torque");
    CHECK_NEAR(achieved.torque_nm.y, 0.0f, 1e-3f,
               "pure yaw: no pitch torque appears (this geometry structurally cannot produce "
               "pitch from differential tilt)");
}

static void test_pitch_unachievable_with_zero_com_offset()
{
    // The central finding of docs/control_allocation.md: with the default
    // center_of_mass_offset_m == Vec3::Zero(), this vehicle's geometry has NO pitch-torque
    // authority near hover. A pitch-only command must be honestly reported as unachievable
    // (saturated == true), not silently "achieved" with zero actual effect.
    const VehicleParams params = baseParams();
    const ControlAllocator allocator(params);
    const float T = hoverThrustN(params);
    const Vec3 desired_torque(0.0f, 0.3f, 0.0f);

    const AllocatedCommand cmd = allocator.allocate(T, desired_torque);
    CHECK(cmd.saturated,
          "pitch with zero CoM offset: unachievable pitch demand is reported as saturated");

    const AchievedEffect achieved = feedThroughForwardModel(params, cmd);
    CHECK_NEAR(achieved.torque_nm.y, 0.0f, 1e-3f,
               "pitch with zero CoM offset: the forward model confirms ~zero pitch torque is "
               "actually achieved, matching the documented structural limitation");
}

static void test_pitch_achievable_with_com_offset()
{
    // Same pitch-only command as above, but with a nonzero center_of_mass_offset_m.z (the
    // physically-necessary condition for pitch authority) -- now it should actually work.
    const VehicleParams params = comOffsetParams();
    const ControlAllocator allocator(params);
    const float T = hoverThrustN(params);
    const Vec3 desired_torque(0.0f, 0.15f, 0.0f);

    const AllocatedCommand cmd = allocator.allocate(T, desired_torque);
    CHECK(!cmd.saturated, "pitch with nonzero CoM offset: modest pitch demand is achievable");
    CHECK_NEAR(cmd.motor1_tilt_rad, cmd.motor2_tilt_rad, 1e-3f,
               "pitch with nonzero CoM offset: pure pitch is a COMMON-mode tilt response, "
               "unlike yaw's differential-tilt response");

    const AchievedEffect achieved = feedThroughForwardModel(params, cmd);
    CHECK_NEAR(achieved.torque_nm.y, desired_torque.y, 1.5e-2f,
               "pitch with nonzero CoM offset: forward model reproduces the requested pitch "
               "torque");
}

static void test_round_trip_combined_thrust_roll_pitch_yaw()
{
    // The single most important test in this milestone (per the milestone brief): a combined,
    // multi-axis, non-degenerate command, round-tripped through the REAL Milestone 9 forward
    // model. Uses comOffsetParams() so all three torque axes are simultaneously achievable.
    const VehicleParams params = comOffsetParams();
    const ControlAllocator allocator(params);
    const float T = hoverThrustN(params);
    const Vec3 desired_torque(0.5f, 0.15f, 0.2f);

    const AllocatedCommand cmd = allocator.allocate(T, desired_torque);
    CHECK(!cmd.saturated, "combined round-trip: this command is within the configured envelope");
    checkCommandInBounds(params, allocator.config(), cmd, "combined round-trip");

    const AchievedEffect achieved = feedThroughForwardModel(params, cmd);
    // A looser tolerance than the isolated-axis tests above: this command combines nonzero
    // roll/pitch/yaw simultaneously, so the individual motor tilts are larger (still a modest
    // ~15 degrees) and the small-angle linearization's known O(theta^2) error compounds across
    // axes -- see docs/control_allocation.md's "Accuracy and its limits" section. 8% relative
    // tolerance comfortably covers that at this command's magnitude while still meaningfully
    // testing that the inverse is correct, not just roughly plausible.
    CHECK_NEAR(achieved.thrust_n, T, 0.08f * T,
               "combined round-trip: achieved thrust matches request within linearization error");
    CHECK_NEAR(achieved.torque_nm.x, desired_torque.x, 0.08f * std::fabs(desired_torque.x) + 0.01f,
               "combined round-trip: achieved roll torque matches request");
    CHECK_NEAR(achieved.torque_nm.y, desired_torque.y, 0.08f * std::fabs(desired_torque.y) + 0.01f,
               "combined round-trip: achieved pitch torque matches request");
    CHECK_NEAR(achieved.torque_nm.z, desired_torque.z, 0.08f * std::fabs(desired_torque.z) + 0.01f,
               "combined round-trip: achieved yaw torque matches request");
}

static void test_saturation_excess_thrust_clamps_to_envelope()
{
    const VehicleParams params = baseParams();
    const ControlAllocator allocator(params);

    const AllocatedCommand cmd = allocator.allocate(1000.0f, Vec3::Zero());
    CHECK(cmd.saturated, "excess thrust: reported as saturated");
    checkCommandInBounds(params, allocator.config(), cmd, "excess thrust");
    CHECK_NEAR(cmd.motor1_throttle, 1.0f, 1e-4f,
               "excess thrust: motor1 throttle clamps to its configured max");
    CHECK_NEAR(cmd.motor2_throttle, 1.0f, 1e-4f,
               "excess thrust: motor2 throttle clamps to its configured max");
}

static void test_saturation_negative_thrust_clamps_to_zero()
{
    const VehicleParams params = baseParams();
    const ControlAllocator allocator(params);

    const AllocatedCommand cmd = allocator.allocate(-50.0f, Vec3::Zero());
    CHECK(cmd.saturated, "negative thrust: reported as saturated");
    checkCommandInBounds(params, allocator.config(), cmd, "negative thrust");
    CHECK_NEAR(cmd.motor1_throttle, 0.0f, 1e-4f, "negative thrust: motor1 throttle clamps to 0");
    CHECK_NEAR(cmd.motor2_throttle, 0.0f, 1e-4f, "negative thrust: motor2 throttle clamps to 0");
}

static void test_saturation_extreme_yaw_clamps_tilt()
{
    const VehicleParams params = baseParams();
    const ControlAllocator allocator(params);
    const float T = hoverThrustN(params);

    const AllocatedCommand cmd = allocator.allocate(T, Vec3(0.0f, 0.0f, 100.0f));
    CHECK(cmd.saturated, "extreme yaw: reported as saturated");
    checkCommandInBounds(params, allocator.config(), cmd, "extreme yaw");
    const float max_tilt = params.motor1.max_tilt_rad;
    CHECK(std::fabs(cmd.motor1_tilt_rad) >= max_tilt - 1e-3f ||
              std::fabs(cmd.motor2_tilt_rad) >= max_tilt - 1e-3f,
          "extreme yaw: at least one motor's tilt saturates at its configured limit");
}

static void test_saturation_extreme_roll_clamps_and_never_produces_garbage()
{
    const VehicleParams params = baseParams();
    const ControlAllocator allocator(params);
    const float T = hoverThrustN(params);

    const AllocatedCommand cmd = allocator.allocate(T, Vec3(100.0f, 0.0f, 0.0f));
    CHECK(cmd.saturated, "extreme roll: reported as saturated");
    checkCommandInBounds(params, allocator.config(), cmd, "extreme roll");
    // Both motors get driven toward opposite throttle extremes rather than producing a negative
    // or >1 throttle (the pre-clamp/pre-bounds-checked failure mode this milestone must avoid).
    CHECK(cmd.motor1_throttle > cmd.motor2_throttle,
          "extreme roll: motor1 still gets more throttle than motor2 in the demanded direction");
}

static void test_never_produces_nan_or_inf_for_adversarial_inputs()
{
    const VehicleParams params = baseParams();
    const ControlAllocator allocator(params);

    const float thrusts[] = {-1e6f, 0.0f, 1e6f, hoverThrustN(params)};
    const Vec3 torques[] = {Vec3(1e6f, 1e6f, 1e6f), Vec3(-1e6f, -1e6f, -1e6f), Vec3::Zero(),
                             Vec3(1e6f, -1e6f, 1e6f)};
    for (float t : thrusts) {
        for (const Vec3& tau : torques) {
            const AllocatedCommand cmd = allocator.allocate(t, tau);
            CHECK(std::isfinite(cmd.motor1_throttle) && std::isfinite(cmd.motor2_throttle) &&
                      std::isfinite(cmd.motor1_tilt_rad) && std::isfinite(cmd.motor2_tilt_rad),
                  "adversarial input: allocator output is always finite, never NaN/Inf");
            checkCommandInBounds(params, allocator.config(), cmd, "adversarial input");
        }
    }
}

static void test_degenerate_zero_arm_offset_falls_back_without_crashing()
{
    // Both motors at the same Y position: roll (and, with zero CoM offset, pitch/yaw's fallback
    // determinant too) is genuinely unachievable. This must degrade gracefully, not divide by
    // zero.
    VehicleParams params = baseParams();
    params.motor1.arm_offset_y_m = 0.0f;
    params.motor2.arm_offset_y_m = 0.0f;
    const ControlAllocator allocator(params);
    const float T = hoverThrustN(params);

    const AllocatedCommand cmd = allocator.allocate(T, Vec3(0.5f, 0.3f, 0.2f));
    CHECK(std::isfinite(cmd.motor1_throttle) && std::isfinite(cmd.motor2_throttle) &&
              std::isfinite(cmd.motor1_tilt_rad) && std::isfinite(cmd.motor2_tilt_rad),
          "degenerate geometry: output stays finite instead of NaN from a zero determinant");
    CHECK(cmd.saturated, "degenerate geometry: roll/pitch/yaw demand is reported as unachievable");
    CHECK_NEAR(cmd.motor1_throttle, cmd.motor2_throttle, 1e-4f,
               "degenerate geometry: falls back to an even thrust split");
}

int main()
{
    test_hover_symmetric_zero_tilt();
    test_pure_roll_produces_differential_thrust();
    test_pure_yaw_produces_differential_tilt();
    test_pitch_unachievable_with_zero_com_offset();
    test_pitch_achievable_with_com_offset();
    test_round_trip_combined_thrust_roll_pitch_yaw();
    test_saturation_excess_thrust_clamps_to_envelope();
    test_saturation_negative_thrust_clamps_to_zero();
    test_saturation_extreme_yaw_clamps_tilt();
    test_saturation_extreme_roll_clamps_and_never_produces_garbage();
    test_never_produces_nan_or_inf_for_adversarial_inputs();
    test_degenerate_zero_arm_offset_falls_back_without_crashing();

    std::printf("%d/%d checks passed\n", g_checks - g_failures, g_checks);
    return g_failures == 0 ? 0 : 1;
}
