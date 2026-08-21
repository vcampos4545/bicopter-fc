// Real automated tests for flight_core/control/rate_controller.{h,cpp}: axis independence (a
// roll-rate error alone must produce ~zero pitch/yaw torque, and so on for each axis), and that
// each axis's response tracks a standalone Pid instance configured identically. Same hand-rolled
// assert-and-report harness as tests/pid_test.cpp.

#include <cmath>
#include <cstdio>

#include "rate_controller.h"

using bicopter::PidConfig;
using bicopter::RateController;
using bicopter::RateControllerConfig;
using bicopter::Vec3;

static int g_failures = 0;
static int g_checks = 0;

#define CHECK(cond, msg)                                                                         \
    do {                                                                                          \
        g_checks++;                                                                               \
        if (!(cond)) {                                                                            \
            g_failures++;                                                                         \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, msg);                             \
        }                                                                                          \
    } while (0)

#define CHECK_NEAR(a, b, tol, msg) CHECK(std::fabs((a) - (b)) <= (tol), msg)

static PidConfig SimpleAxisConfig()
{
    PidConfig config;
    config.kp = 0.5f;
    config.ki = 0.1f;
    config.kd = 0.02f;
    config.output_min = -10.0f;
    config.output_max = 10.0f;
    config.derivative_on_measurement = true;
    return config;
}

static void test_axis_independence()
{
    RateControllerConfig config;
    config.roll = SimpleAxisConfig();
    config.pitch = SimpleAxisConfig();
    config.yaw = SimpleAxisConfig();
    RateController controller(config);

    // Roll-rate error alone: desired roll rate != current roll rate, pitch/yaw desired == current.
    const Vec3 desired_roll_only(1.0f, 0.0f, 0.0f);
    const Vec3 current_zero(0.0f, 0.0f, 0.0f);
    const Vec3 torque_roll = controller.update(desired_roll_only, current_zero, 0.02f);

    CHECK(torque_roll.x != 0.0f, "roll-rate error alone produces nonzero roll torque");
    CHECK_NEAR(torque_roll.y, 0.0f, 1e-6f, "roll-rate error alone produces ~zero pitch torque");
    CHECK_NEAR(torque_roll.z, 0.0f, 1e-6f, "roll-rate error alone produces ~zero yaw torque");

    RateController controller2(config);
    const Vec3 desired_pitch_only(0.0f, 1.0f, 0.0f);
    const Vec3 torque_pitch = controller2.update(desired_pitch_only, current_zero, 0.02f);

    CHECK_NEAR(torque_pitch.x, 0.0f, 1e-6f, "pitch-rate error alone produces ~zero roll torque");
    CHECK(torque_pitch.y != 0.0f, "pitch-rate error alone produces nonzero pitch torque");
    CHECK_NEAR(torque_pitch.z, 0.0f, 1e-6f, "pitch-rate error alone produces ~zero yaw torque");

    RateController controller3(config);
    const Vec3 desired_yaw_only(0.0f, 0.0f, 1.0f);
    const Vec3 torque_yaw = controller3.update(desired_yaw_only, current_zero, 0.02f);

    CHECK_NEAR(torque_yaw.x, 0.0f, 1e-6f, "yaw-rate error alone produces ~zero roll torque");
    CHECK_NEAR(torque_yaw.y, 0.0f, 1e-6f, "yaw-rate error alone produces ~zero pitch torque");
    CHECK(torque_yaw.z != 0.0f, "yaw-rate error alone produces nonzero yaw torque");
}

static void test_matches_standalone_pid_per_axis()
{
    RateControllerConfig config;
    config.roll = SimpleAxisConfig();
    config.pitch = SimpleAxisConfig();
    config.yaw = SimpleAxisConfig();
    RateController controller(config);

    bicopter::Pid roll_pid(config.roll);
    bicopter::Pid pitch_pid(config.pitch);
    bicopter::Pid yaw_pid(config.yaw);

    // Drive both the RateController and three independent standalone Pid instances with the same
    // sequence of rate errors and confirm they agree exactly, across several steps (so integral/
    // derivative history is exercised too, not just a single step).
    const Vec3 desired(1.5f, -0.8f, 0.3f);
    const float dt = 0.02f;
    Vec3 current(0.0f, 0.0f, 0.0f);

    for (int i = 0; i < 5; i++) {
        const Vec3 torque = controller.update(desired, current, dt);

        const float expect_x = roll_pid.update(desired.x, current.x, dt);
        const float expect_y = pitch_pid.update(desired.y, current.y, dt);
        const float expect_z = yaw_pid.update(desired.z, current.z, dt);

        CHECK_NEAR(torque.x, expect_x, 1e-5f, "roll torque matches a standalone Pid on the same axis");
        CHECK_NEAR(torque.y, expect_y, 1e-5f, "pitch torque matches a standalone Pid on the same axis");
        CHECK_NEAR(torque.z, expect_z, 1e-5f, "yaw torque matches a standalone Pid on the same axis");

        // Current rate drifts a bit each step, just to exercise changing measurements too.
        current = current + Vec3(0.1f, -0.05f, 0.02f);
    }
}

static void test_reset_clears_all_axes()
{
    RateControllerConfig config;
    config.roll = SimpleAxisConfig();
    config.pitch = SimpleAxisConfig();
    config.yaw = SimpleAxisConfig();
    RateController controller(config);

    const Vec3 desired(1.0f, 1.0f, 1.0f);
    const Vec3 current(0.0f, 0.0f, 0.0f);
    for (int i = 0; i < 5; i++) {
        controller.update(desired, current, 0.02f);
    }

    controller.reset();

    RateController fresh(config);
    const Vec3 torque_after_reset = controller.update(desired, current, 0.02f);
    const Vec3 torque_fresh = fresh.update(desired, current, 0.02f);

    CHECK_NEAR(torque_after_reset.x, torque_fresh.x, 1e-5f, "reset() makes roll axis behave fresh");
    CHECK_NEAR(torque_after_reset.y, torque_fresh.y, 1e-5f, "reset() makes pitch axis behave fresh");
    CHECK_NEAR(torque_after_reset.z, torque_fresh.z, 1e-5f, "reset() makes yaw axis behave fresh");
}

int main()
{
    test_axis_independence();
    test_matches_standalone_pid_per_axis();
    test_reset_clears_all_axes();

    std::printf("%d/%d checks passed\n", g_checks - g_failures, g_checks);
    return g_failures == 0 ? 0 : 1;
}
