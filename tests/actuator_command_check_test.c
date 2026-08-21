// Real automated tests for the defense-in-depth actuator-command validator
// (actuator_command_check.c). Same hand-rolled harness as the rest of this project's host tests.
#include <math.h>
#include <stdio.h>

#include "actuator_command_check.h"

static int g_failures = 0;
static int g_checks = 0;

#define CHECK(cond, msg)                                                                       \
    do {                                                                                        \
        g_checks++;                                                                             \
        if (!(cond)) {                                                                          \
            g_failures++;                                                                       \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, msg);                                \
        }                                                                                        \
    } while (0)

static actuator_command_limits_t test_limits(void)
{
    actuator_command_limits_t limits = {
        .min_throttle = 0.0f,
        .max_throttle = 1.0f,
        .min_tilt_rad = -0.5f,
        .max_tilt_rad = 0.5f,
    };
    return limits;
}

static actuator_command_t nominal_command(void)
{
    actuator_command_t cmd = {
        .motor1_throttle = 0.5f,
        .motor2_throttle = 0.5f,
        .servo1_tilt_rad = 0.0f,
        .servo2_tilt_rad = 0.0f,
    };
    return cmd;
}

static void test_nominal_command_is_valid(void)
{
    actuator_command_limits_t limits = test_limits();
    actuator_command_t cmd = nominal_command();
    CHECK(actuator_command_is_valid(&cmd, &limits), "a nominal, in-range command is valid");
}

static void test_boundary_values_are_valid(void)
{
    actuator_command_limits_t limits = test_limits();
    actuator_command_t cmd = {
        .motor1_throttle = limits.min_throttle,
        .motor2_throttle = limits.max_throttle,
        .servo1_tilt_rad = limits.min_tilt_rad,
        .servo2_tilt_rad = limits.max_tilt_rad,
    };
    CHECK(actuator_command_is_valid(&cmd, &limits), "exact boundary values are valid (inclusive)");
}

static void test_out_of_range_throttle_is_invalid(void)
{
    actuator_command_limits_t limits = test_limits();

    actuator_command_t over = nominal_command();
    over.motor1_throttle = 1.5f;
    CHECK(!actuator_command_is_valid(&over, &limits), "motor1 throttle above max is invalid");

    actuator_command_t under = nominal_command();
    under.motor2_throttle = -0.1f;
    CHECK(!actuator_command_is_valid(&under, &limits), "motor2 throttle below min is invalid");
}

static void test_out_of_range_tilt_is_invalid(void)
{
    actuator_command_limits_t limits = test_limits();

    actuator_command_t over = nominal_command();
    over.servo1_tilt_rad = 0.6f;
    CHECK(!actuator_command_is_valid(&over, &limits), "servo1 tilt above max is invalid");

    actuator_command_t under = nominal_command();
    under.servo2_tilt_rad = -0.6f;
    CHECK(!actuator_command_is_valid(&under, &limits), "servo2 tilt below min is invalid");
}

static void test_non_finite_values_are_invalid_regardless_of_limits(void)
{
    actuator_command_limits_t limits = test_limits();

    actuator_command_t nan_cmd = nominal_command();
    nan_cmd.motor1_throttle = NAN;
    CHECK(!actuator_command_is_valid(&nan_cmd, &limits), "NaN throttle is always invalid");

    actuator_command_t inf_cmd = nominal_command();
    inf_cmd.servo1_tilt_rad = INFINITY;
    CHECK(!actuator_command_is_valid(&inf_cmd, &limits), "+Inf tilt is always invalid");

    actuator_command_t neg_inf_cmd = nominal_command();
    neg_inf_cmd.motor2_throttle = -INFINITY;
    CHECK(!actuator_command_is_valid(&neg_inf_cmd, &limits), "-Inf throttle is always invalid");
}

static void test_swapped_limits_are_handled_defensively(void)
{
    // A misconfigured limits struct with min > max must not silently accept everything.
    actuator_command_limits_t limits = {
        .min_throttle = 1.0f,
        .max_throttle = 0.0f,
        .min_tilt_rad = 0.5f,
        .max_tilt_rad = -0.5f,
    };
    actuator_command_t cmd = nominal_command(); // 0.5 throttle, 0.0 tilt - within the sorted range
    CHECK(actuator_command_is_valid(&cmd, &limits),
          "swapped min/max limits are sorted defensively, not treated as an empty range");
}

static void test_null_pointers_are_invalid(void)
{
    actuator_command_limits_t limits = test_limits();
    actuator_command_t cmd = nominal_command();
    CHECK(!actuator_command_is_valid(NULL, &limits), "a NULL command is invalid");
    CHECK(!actuator_command_is_valid(&cmd, NULL), "a NULL limits pointer is invalid");
}

int main(void)
{
    test_nominal_command_is_valid();
    test_boundary_values_are_valid();
    test_out_of_range_throttle_is_invalid();
    test_out_of_range_tilt_is_invalid();
    test_non_finite_values_are_invalid_regardless_of_limits();
    test_swapped_limits_are_handled_defensively();
    test_null_pointers_are_invalid();

    printf("%d/%d checks passed\n", g_checks - g_failures, g_checks);
    return g_failures == 0 ? 0 : 1;
}
