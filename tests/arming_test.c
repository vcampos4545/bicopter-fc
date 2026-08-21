// Real automated tests for arming-precondition logic (arming.c): each precondition individually
// blocking arming, the all-clear case allowing it, the blocking-mask bit assignment, and the
// stationary-check math. Same hand-rolled harness as the rest of this project's host tests.
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "arming.h"

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

static arming_preconditions_t all_clear(void)
{
    arming_preconditions_t p;
    memset(&p, 0, sizeof(p));
    p.imu_valid = true;
    p.estimator_valid = true;
    p.radio_link_alive = true;
    p.radio_arm_command = true;
    p.battery_ok = true;
    p.no_critical_errors = true;
    p.is_stationary = true;
    return p;
}

static void test_all_clear_allows_arming(void)
{
    arming_preconditions_t p = all_clear();
    CHECK(arming_preconditions_met(&p), "every precondition true allows arming");
    CHECK(arming_blocking_mask(&p) == 0, "no blocking bits when every precondition holds");
}

// Table-driven: flips exactly one field false at a time and confirms it (a) blocks arming and
// (b) sets exactly its own bit in the blocking mask, with every other field still passing.
#define FIELD_CASE(field, flag) {offsetof(arming_preconditions_t, field), (flag), #field}

struct field_case {
    size_t offset;
    uint32_t expected_flag;
    const char *name;
};

static void test_each_precondition_individually_blocks(void)
{
    static const struct field_case cases[] = {
        FIELD_CASE(imu_valid, ARMING_BLOCK_IMU),
        FIELD_CASE(estimator_valid, ARMING_BLOCK_ESTIMATOR),
        FIELD_CASE(radio_link_alive, ARMING_BLOCK_RADIO_LINK),
        FIELD_CASE(radio_arm_command, ARMING_BLOCK_RADIO_ARM_CMD),
        FIELD_CASE(battery_ok, ARMING_BLOCK_BATTERY),
        FIELD_CASE(no_critical_errors, ARMING_BLOCK_CRITICAL_ERROR),
        FIELD_CASE(is_stationary, ARMING_BLOCK_NOT_STATIONARY),
    };

    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        arming_preconditions_t p = all_clear();
        bool *field = (bool *)((char *)&p + cases[i].offset);
        *field = false;

        CHECK(!arming_preconditions_met(&p), cases[i].name);
        CHECK(arming_blocking_mask(&p) == cases[i].expected_flag, cases[i].name);
    }
}

static void test_multiple_failures_combine_in_mask(void)
{
    arming_preconditions_t p = all_clear();
    p.imu_valid = false;
    p.battery_ok = false;

    CHECK(!arming_preconditions_met(&p), "multiple failing preconditions still block arming");
    CHECK(arming_blocking_mask(&p) == (ARMING_BLOCK_IMU | ARMING_BLOCK_BATTERY),
          "blocking mask ORs together every failing precondition's bit");
}

static void test_null_input_fails_closed(void)
{
    CHECK(!arming_preconditions_met(NULL), "a NULL input never reports arming allowed");
    CHECK(arming_blocking_mask(NULL) != 0, "a NULL input reports every bit blocked, not zero");
}

static void test_is_stationary(void)
{
    CHECK(arming_is_stationary(0.0f, 0.0f, 0.0f, 0.1f), "zero rate is always stationary");
    CHECK(arming_is_stationary(0.05f, 0.0f, 0.0f, 0.1f), "rate below threshold is stationary");
    CHECK(!arming_is_stationary(0.2f, 0.0f, 0.0f, 0.1f),
          "single-axis rate above threshold is not stationary");
    CHECK(!arming_is_stationary(0.08f, 0.08f, 0.0f, 0.1f),
          "combined multi-axis magnitude above threshold is not stationary, even if each axis "
          "individually is below it");

    // Threshold is checked via magnitude, not per-axis: exactly at the threshold.
    CHECK(arming_is_stationary(0.1f, 0.0f, 0.0f, 0.1f), "exactly-at-threshold counts as stationary");

    // Non-positive threshold disables the check.
    CHECK(arming_is_stationary(1000.0f, 1000.0f, 1000.0f, 0.0f),
          "a non-positive threshold disables the stationary check (always true)");
    CHECK(arming_is_stationary(1000.0f, 1000.0f, 1000.0f, -1.0f),
          "a negative threshold also disables the stationary check");
}

int main(void)
{
    test_all_clear_allows_arming();
    test_each_precondition_individually_blocks();
    test_multiple_failures_combine_in_mask();
    test_null_input_fails_closed();
    test_is_stationary();

    printf("%d/%d checks passed\n", g_checks - g_failures, g_checks);
    return g_failures == 0 ? 0 : 1;
}
