// Real automated tests for simulator/physics/bicopter_dynamics.{h,cpp}: free-fall kinematics,
// symmetric-thrust hover equilibrium (zero net acceleration and torque), asymmetric-thrust and
// tilt-induced torque (checked against a hand-derived closed form of the same r x F formula
// under test), reaction-torque-driven yaw, and a torque-free angular-momentum conservation check.
// Same hand-rolled assert-and-report harness as tests/quaternion_test.cpp. See docs/dynamics.md
// for the full derivation every expected value below is computed from.

#include <cmath>
#include <cstdio>

#include "bicopter_dynamics.h"

using bicopter::ActuatorCommand;
using bicopter::MotorParams;
using bicopter::RigidBodyState;
using bicopter::StateDerivative;
using bicopter::VehicleParams;
using bicopter::Vec3;

namespace {
constexpr float kPi = 3.14159265358979323846f;
// Must match bicopter_dynamics.cpp's internal kGravityMps2 (standard gravity, same value as
// flight_core/estimation's ComplementaryFilterConfig::gravity_mps2 default).
constexpr float kGravityMps2 = 9.80665f;
} // namespace

static int g_failures = 0;
static int g_checks = 0;

#define CHECK(cond, msg)                                                                        \
    do {                                                                                         \
        g_checks++;                                                                              \
        if (!(cond)) {                                                                           \
            g_failures++;                                                                        \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, msg);                            \
        }                                                                                         \
    } while (0)

#define CHECK_NEAR(a, b, tol, msg) CHECK(std::fabs((a) - (b)) <= (tol), msg)

static bool vec_near(const Vec3& a, const Vec3& b, float tol)
{
    return std::fabs(a.x - b.x) <= tol && std::fabs(a.y - b.y) <= tol &&
           std::fabs(a.z - b.z) <= tol;
}

// A vehicle config with no thrust/torque authority and no drag -- the baseline every test starts
// from and overrides only the fields it needs, so each test's intent (and what's held at zero to
// isolate one effect) is visible at the call site.
static VehicleParams baseParams()
{
    VehicleParams p;
    p.mass_kg = 2.0f;
    p.inertia_diag_kg_m2 = Vec3(0.02f, 0.03f, 0.04f); // deliberately asymmetric
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

static void test_thrust_direction()
{
    const Vec3 up = bicopter::motorThrustDirectionBody(0.0f);
    CHECK(vec_near(up, Vec3(0.0f, 0.0f, -1.0f), 1e-6f), "zero tilt points straight up (-Z, body)");

    const Vec3 quarter = bicopter::motorThrustDirectionBody(kPi / 2.0f);
    CHECK(vec_near(quarter, Vec3(-1.0f, 0.0f, 0.0f), 1e-5f),
          "90-deg tilt rotates thrust to -X per the documented right-hand-about-+Y convention");

    for (float t = -1.2f; t <= 1.2f; t += 0.3f) {
        const Vec3 dir = bicopter::motorThrustDirectionBody(t);
        CHECK_NEAR(dir.length(), 1.0f, 1e-6f, "thrust direction is always a unit vector");
        CHECK_NEAR(dir.y, 0.0f, 1e-6f, "tilt about +Y never produces a body-Y thrust component");
    }
}

static void test_free_fall()
{
    // Zero thrust, zero drag: pure gravity. m*v_dot=F reduces to v_dot=(0,0,g) (NED, +Z down).
    VehicleParams params = baseParams();
    RigidBodyState state; // starts at rest at the origin, identity orientation
    const ActuatorCommand zero_command; // both throttles 0

    const float dt = 0.001f;
    const int steps = 1000; // 1 second total
    for (int i = 0; i < steps; ++i) {
        state = bicopter::stepRigidBodyState(state, params, zero_command, dt);
    }

    // Semi-implicit Euler makes velocity exact for constant acceleration (each step adds g*dt).
    CHECK_NEAR(state.velocity_mps.z, kGravityMps2 * 1.0f, 1e-4f,
               "free fall: v_z after 1s matches g*t exactly (semi-implicit Euler is exact for "
               "constant acceleration)");
    CHECK_NEAR(state.velocity_mps.x, 0.0f, 1e-6f, "free fall: no horizontal velocity");
    CHECK_NEAR(state.velocity_mps.y, 0.0f, 1e-6f, "free fall: no horizontal velocity");

    // Position has a small, well-understood O(dt) discretization bias above the analytic
    // 0.5*g*t^2 -- see docs/dynamics.md's "Integration scheme" section. 1% tolerance comfortably
    // covers it at this step size.
    const float expected_z = 0.5f * kGravityMps2 * 1.0f * 1.0f;
    CHECK_NEAR(state.position_m.z, expected_z, 0.01f * expected_z,
               "free fall: z position matches 0.5*g*t^2 within semi-implicit Euler's O(dt) bias");

    CHECK_NEAR(state.angular_velocity_radps.length(), 0.0f, 1e-6f,
               "free fall: no torque means no rotation");
    CHECK_NEAR(state.orientation.w, 1.0f, 1e-6f, "free fall: orientation stays identity");
}

static void test_hover_equilibrium()
{
    // Symmetric equal thrust, zero tilt, total thrust == weight: m*v_dot=F should give ~zero net
    // acceleration, and the symmetric r x F terms plus the opposite-spin reaction torques should
    // give ~zero net torque.
    VehicleParams params = baseParams();
    params.motor1.torque_coefficient_nm_per_n = 0.02f; // nonzero so cancellation is exercised
    params.motor2.torque_coefficient_nm_per_n = 0.02f;

    const float half_weight_n = 0.5f * params.mass_kg * kGravityMps2;
    const float hover_cmd = std::sqrt(half_weight_n / params.motor1.thrust_coefficient_n);

    RigidBodyState state;
    ActuatorCommand command;
    command.motor1_throttle = hover_cmd;
    command.motor2_throttle = hover_cmd;

    const StateDerivative deriv = bicopter::computeStateDerivative(state, params, command);

    CHECK(vec_near(deriv.acceleration_mps2, Vec3::Zero(), 1e-4f),
          "hover: equal-and-opposite thrust vs. gravity gives ~zero net acceleration");
    CHECK(vec_near(deriv.angular_acceleration_radps2, Vec3::Zero(), 1e-4f),
          "hover: symmetric geometry + opposite motor spin gives ~zero net torque");
}

static void test_asymmetric_thrust_induces_roll_torque()
{
    // Differential thrust at zero tilt, reaction torque disabled, isolates r x F. For motors at
    // y=-L/y=+L with thrust t1/t2 along body -Z: r1 x F1 = (L*t1, 0, 0), r2 x F2 = (-L*t2, 0, 0)
    // -- see docs/dynamics.md's derivation. Net torque is pure roll: (L*(t1-t2), 0, 0).
    VehicleParams params = baseParams();
    const float L = 0.15f;
    const float t1 = 8.0f, t2 = 5.0f; // Newtons
    const float cmd1 = std::sqrt(t1 / params.motor1.thrust_coefficient_n);
    const float cmd2 = std::sqrt(t2 / params.motor2.thrust_coefficient_n);

    RigidBodyState state;
    ActuatorCommand command;
    command.motor1_throttle = cmd1;
    command.motor2_throttle = cmd2;

    const StateDerivative deriv = bicopter::computeStateDerivative(state, params, command);

    const float expected_tau_x = L * (t1 - t2);
    const Vec3 expected_alpha(expected_tau_x / params.inertia_diag_kg_m2.x, 0.0f, 0.0f);

    CHECK(vec_near(deriv.angular_acceleration_radps2, expected_alpha, 1e-3f),
          "asymmetric thrust produces roll torque matching the hand-derived r x F formula");
}

static void test_tilt_induces_yaw_and_roll_torque()
{
    // Single tilted, offset motor, reaction torque disabled, isolates r x F under a nonzero tilt.
    // r=(0,L,0), F=t*(-sin(tilt),0,-cos(tilt)); r x F = (L*Fz, 0, -L*Fx)
    //          = (-L*t*cos(tilt), 0, L*t*sin(tilt)) -- see docs/dynamics.md.
    VehicleParams params = baseParams();
    const float L = 0.15f;
    const float t = 6.0f;
    const float tilt = 0.3f;
    const float cmd = std::sqrt(t / params.motor2.thrust_coefficient_n);

    RigidBodyState state;
    ActuatorCommand command;
    command.motor1_throttle = 0.0f;
    command.motor2_throttle = cmd;
    command.motor2_tilt_rad = tilt;

    const StateDerivative deriv = bicopter::computeStateDerivative(state, params, command);

    const Vec3 expected_tau(-L * t * std::cos(tilt), 0.0f, L * t * std::sin(tilt));
    const Vec3 expected_alpha(expected_tau.x / params.inertia_diag_kg_m2.x, 0.0f,
                               expected_tau.z / params.inertia_diag_kg_m2.z);

    CHECK(vec_near(deriv.angular_acceleration_radps2, expected_alpha, 1e-3f),
          "tilted offset motor produces both roll and yaw torque matching the hand-derived "
          "r x F formula");
}

static void test_reaction_torque_yaw()
{
    // Motor at the center of mass (r=0, so r x F vanishes) and zero tilt isolates the reaction
    // torque term, which acts about the motor's spin axis -- (0,0,-1) at zero tilt, i.e. pure
    // yaw. Expected tau_z = spin_direction * torque_coefficient * thrust * (-1).
    VehicleParams params = baseParams();
    params.motor1.arm_offset_y_m = 0.0f;
    params.motor2.arm_offset_y_m = 0.0f;
    params.motor2.torque_coefficient_nm_per_n = 0.03f;
    params.motor2.spin_direction = 1.0f;

    const float t = 7.0f;
    const float cmd = std::sqrt(t / params.motor2.thrust_coefficient_n);

    RigidBodyState state;
    ActuatorCommand command;
    command.motor2_throttle = cmd;

    const StateDerivative deriv = bicopter::computeStateDerivative(state, params, command);

    const float expected_tau_z = -params.motor2.spin_direction * params.motor2.torque_coefficient_nm_per_n * t;
    CHECK_NEAR(deriv.angular_acceleration_radps2.z, expected_tau_z / params.inertia_diag_kg_m2.z,
               1e-3f, "motor reaction torque shows up as yaw angular acceleration");
    CHECK_NEAR(deriv.angular_acceleration_radps2.x, 0.0f, 1e-4f,
               "reaction torque alone produces no roll");
    CHECK_NEAR(deriv.angular_acceleration_radps2.y, 0.0f, 1e-4f,
               "reaction torque alone produces no pitch");
}

static void test_torque_free_steady_spin_about_principal_axis()
{
    // Zero commands, zero drag: no external torque. Spinning purely about a principal axis of a
    // diagonal inertia tensor is a steady state -- omega x (I*omega) vanishes exactly, so
    // angular momentum direction (and magnitude) must stay constant. See docs/dynamics.md's
    // "Tests" section.
    VehicleParams params = baseParams();
    params.motor1.thrust_coefficient_n = 0.0f;
    params.motor2.thrust_coefficient_n = 0.0f;

    RigidBodyState state;
    state.angular_velocity_radps = Vec3(0.0f, 0.0f, 5.0f); // pure yaw-axis spin
    const ActuatorCommand zero_command;

    const StateDerivative deriv = bicopter::computeStateDerivative(state, params, zero_command);
    CHECK(vec_near(deriv.angular_acceleration_radps2, Vec3::Zero(), 1e-6f),
          "torque-free spin about a principal axis produces zero angular acceleration");
}

static void test_torque_free_angular_momentum_magnitude_conserved()
{
    // A non-principal-axis initial spin, integrated for many torque-free steps: |I .* omega|
    // (angular momentum magnitude) is a conserved quantity of torque-free rigid-body rotation
    // and should stay ~constant over the trajectory even as the direction of omega itself
    // precesses (Euler's equations / Poinsot's construction).
    VehicleParams params = baseParams();
    params.motor1.thrust_coefficient_n = 0.0f;
    params.motor2.thrust_coefficient_n = 0.0f;

    RigidBodyState state;
    state.angular_velocity_radps = Vec3(3.0f, 1.5f, 2.0f); // not aligned to any principal axis
    const ActuatorCommand zero_command;

    const Vec3& i = params.inertia_diag_kg_m2;
    const auto angularMomentum = [&](const Vec3& omega) {
        return Vec3(i.x * omega.x, i.y * omega.y, i.z * omega.z);
    };
    const float initial_l = angularMomentum(state.angular_velocity_radps).length();

    const float dt = 0.0005f;
    for (int step = 0; step < 2000; ++step) { // 1 second
        state = bicopter::stepRigidBodyState(state, params, zero_command, dt);
        const float l = angularMomentum(state.angular_velocity_radps).length();
        CHECK(std::fabs(l - initial_l) <= 0.01f * initial_l,
              "angular momentum magnitude stays within 1% of its initial value throughout a "
              "torque-free trajectory");
    }
}

int main()
{
    test_thrust_direction();
    test_free_fall();
    test_hover_equilibrium();
    test_asymmetric_thrust_induces_roll_torque();
    test_tilt_induces_yaw_and_roll_torque();
    test_reaction_torque_yaw();
    test_torque_free_steady_spin_about_principal_axis();
    test_torque_free_angular_momentum_magnitude_conserved();

    std::printf("%d/%d checks passed\n", g_checks - g_failures, g_checks);
    return g_failures == 0 ? 0 : 1;
}
