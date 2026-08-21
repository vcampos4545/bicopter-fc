// Real automated tests for flight_core/control/pid.{h,cpp}: proportional-only response,
// integral accumulation, anti-windup under saturation, derivative response (including the
// derivative-on-measurement-avoids-kick behavior), and output clamping. Same hand-rolled
// assert-and-report harness as tests/vec3_test.cpp (no external test framework available offline
// in this environment), exiting non-zero on any failure so it works as a real CI gate.

#include <cmath>
#include <cstdio>

#include "pid.h"

using bicopter::Pid;
using bicopter::PidConfig;

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

static void test_proportional_only()
{
    PidConfig config;
    config.kp = 2.0f;
    config.ki = 0.0f;
    config.kd = 0.0f;
    config.output_min = -100.0f;
    config.output_max = 100.0f;
    Pid pid(config);

    // Step error of 3.0 (setpoint - measurement); proportional-only output is exactly kp*error,
    // unaffected by dt since there's no integral/derivative term to depend on it.
    const float out = pid.update(/*setpoint=*/5.0f, /*measurement=*/2.0f, /*dt=*/0.01f);
    CHECK_NEAR(out, 6.0f, 1e-5f, "kp=2, error=3 -> output=6");

    // A second step at the same error should give the identical output — no hidden state leaks
    // into a pure-P controller.
    const float out2 = pid.update(5.0f, 2.0f, 0.01f);
    CHECK_NEAR(out2, 6.0f, 1e-5f, "repeated identical step gives identical P-only output");

    // Negative error.
    const float out3 = pid.update(0.0f, 4.0f, 0.01f);
    CHECK_NEAR(out3, -8.0f, 1e-5f, "kp=2, error=-4 -> output=-8");
}

static void test_integral_accumulation()
{
    PidConfig config;
    config.kp = 0.0f;
    config.ki = 1.0f;
    config.kd = 0.0f;
    config.output_min = -1000.0f;
    config.output_max = 1000.0f;
    Pid pid(config);

    // Constant error of 2.0, dt=0.1s, ki=1: integral grows by error*dt=0.2 each step, output is
    // ki*integral (unsaturated, so no anti-windup kicks in).
    const float dt = 0.1f;
    const float error = 2.0f;
    float expected_integral = 0.0f;
    for (int i = 1; i <= 5; i++) {
        const float out = pid.update(error, 0.0f, dt);
        expected_integral += error * dt;
        CHECK_NEAR(out, expected_integral, 1e-4f, "integral accumulates error*dt each step");
        CHECK_NEAR(pid.integralTerm(), expected_integral, 1e-4f, "integralTerm() matches");
    }

    // Reset clears the accumulator.
    pid.reset();
    CHECK_NEAR(pid.integralTerm(), 0.0f, 1e-6f, "reset() clears the integral accumulator");
    const float out_after_reset = pid.update(error, 0.0f, dt);
    CHECK_NEAR(out_after_reset, error * dt, 1e-5f, "first step after reset starts from zero");
}

static void test_anti_windup_on_saturation()
{
    // Tight output limits so a modest constant error saturates immediately.
    PidConfig config;
    config.kp = 0.0f;
    config.ki = 1.0f;
    config.kd = 0.0f;
    config.output_min = -1.0f;
    config.output_max = 1.0f;
    Pid pid(config);

    const float dt = 0.1f;
    const float error = 5.0f; // large enough that ki*integral saturates after one step

    // First step: integral = 0.5, output = 0.5 (not yet saturated).
    const float out1 = pid.update(error, 0.0f, dt);
    CHECK_NEAR(out1, 0.5f, 1e-4f, "first integrating step below saturation is un-clamped");

    // Second step: unclamped integral would be 1.0, output clamps to 1.0 exactly at the limit —
    // not yet "integrating further into" saturation since it lands exactly on the boundary.
    const float out2 = pid.update(error, 0.0f, dt);
    CHECK_NEAR(out2, 1.0f, 1e-4f, "second step reaches the output ceiling exactly");

    // From here, further steps with the same-sign error must NOT keep growing the integral —
    // anti-windup holds it, so output stays clamped at exactly output_max rather than requiring
    // many steps of unwind once the error reverses.
    const float integral_at_saturation = pid.integralTerm();
    for (int i = 0; i < 10; i++) {
        const float out = pid.update(error, 0.0f, dt);
        CHECK_NEAR(out, 1.0f, 1e-4f, "output stays clamped at output_max, does not exceed it");
    }
    CHECK_NEAR(pid.integralTerm(), integral_at_saturation, 1e-4f,
               "integral does not keep growing while saturated in the same direction as error");

    // Now reverse the error sign hard. If windup had been allowed to accumulate unbounded, the
    // output would stay pinned at output_max for many steps before recovering. With anti-windup,
    // the very next step should already move off the ceiling.
    const float out_reversed = pid.update(-error, 0.0f, dt);
    CHECK(out_reversed < 1.0f, "output leaves the ceiling immediately once error reverses");
}

static void test_derivative_response()
{
    // Derivative-on-measurement (the default): a changing measurement with constant setpoint
    // produces a derivative term of -(measurement - previous_measurement)/dt.
    PidConfig config;
    config.kp = 0.0f;
    config.ki = 0.0f;
    config.kd = 2.0f;
    config.output_min = -1000.0f;
    config.output_max = 1000.0f;
    config.derivative_on_measurement = true;
    Pid pid(config);

    // First call has no previous measurement, so derivative is 0.
    const float out0 = pid.update(0.0f, 1.0f, 0.1f);
    CHECK_NEAR(out0, 0.0f, 1e-5f, "no derivative term on the first update() call");

    // Measurement rises from 1.0 to 1.5 over dt=0.1s -> d(measurement)/dt = 5.0 ->
    // derivative term = -5.0 -> output = kd * -5.0 = -10.0.
    const float out1 = pid.update(0.0f, 1.5f, 0.1f);
    CHECK_NEAR(out1, -10.0f, 1e-3f, "derivative-on-measurement responds to rising measurement");

    // No derivative kick on a setpoint step: measurement held constant, only the setpoint jumps.
    // derivative-on-measurement must produce zero derivative contribution here.
    Pid pid_kick(config);
    pid_kick.update(0.0f, 2.0f, 0.1f);          // seed previous_measurement_
    const float out_step = pid_kick.update(50.0f, 2.0f, 0.1f); // setpoint jumps, measurement doesn't
    CHECK_NEAR(out_step, 0.0f, 1e-4f,
               "derivative-on-measurement gives no kick when only the setpoint steps");

    // Contrast: derivative-on-error DOES kick on the same setpoint step.
    PidConfig error_config = config;
    error_config.derivative_on_measurement = false;
    Pid pid_error(error_config);
    pid_error.update(0.0f, 2.0f, 0.1f); // error = -2.0, seeds previous_error_
    const float out_error_step = pid_error.update(50.0f, 2.0f, 0.1f); // error jumps to 48.0
    CHECK(std::fabs(out_error_step) > 1.0f,
          "derivative-on-error does produce a kick on the same setpoint step");
}

static void test_output_saturation()
{
    PidConfig config;
    config.kp = 100.0f;
    config.ki = 0.0f;
    config.kd = 0.0f;
    config.output_min = -1.0f;
    config.output_max = 1.0f;
    Pid pid(config);

    const float out_high = pid.update(100.0f, 0.0f, 0.1f);
    CHECK_NEAR(out_high, 1.0f, 1e-6f, "large positive error clamps to output_max");
    CHECK(pid.lastUnsaturatedOutput() > 1.0f, "unsaturated output is visible above the clamp");

    const float out_low = pid.update(-100.0f, 0.0f, 0.1f);
    CHECK_NEAR(out_low, -1.0f, 1e-6f, "large negative error clamps to output_min");
    CHECK(pid.lastUnsaturatedOutput() < -1.0f, "unsaturated output is visible below the clamp");
}

static void test_non_positive_dt_is_proportional_only()
{
    PidConfig config;
    config.kp = 3.0f;
    config.ki = 5.0f;
    config.kd = 5.0f;
    config.output_min = -1000.0f;
    config.output_max = 1000.0f;
    Pid pid(config);

    // Seed some history first.
    pid.update(1.0f, 0.0f, 0.1f);

    const float out = pid.update(1.0f, 0.0f, 0.0f);
    CHECK_NEAR(out, 3.0f, 1e-5f, "dt<=0 returns the proportional-only response");

    const float out_neg_dt = pid.update(1.0f, 0.0f, -0.5f);
    CHECK_NEAR(out_neg_dt, 3.0f, 1e-5f, "negative dt also returns the proportional-only response");
}

int main()
{
    test_proportional_only();
    test_integral_accumulation();
    test_anti_windup_on_saturation();
    test_derivative_response();
    test_output_saturation();
    test_non_positive_dt_is_proportional_only();

    std::printf("%d/%d checks passed\n", g_checks - g_failures, g_checks);
    return g_failures == 0 ? 0 : 1;
}
