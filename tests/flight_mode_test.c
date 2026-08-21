// Real automated tests for the flight-mode state machine (flight_mode.c): the full transition
// table BOOT/DISARMED/ARMED/STABILIZE/ALTITUDE_HOLD/FAILSAFE/ERROR, every valid transition this
// milestone's brief documents plus several transitions that must NOT happen (e.g. DISARMED never
// reaching FAILSAFE directly, FAILSAFE never auto-clearing without an explicit disarm). Same
// hand-rolled assert-and-report harness as the rest of this project's host tests (no external
// framework available offline) - see tests/mpu6050_convert_test.c for the pattern this follows.
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "flight_mode.h"

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

static flight_mode_inputs_t base_inputs(void)
{
    flight_mode_inputs_t in;
    memset(&in, 0, sizeof(in));
    in.arm_command = true;
    in.mode_request = FLIGHT_MODE_REQUEST_ARMED_IDLE;
    in.arming_preconditions_met = true;
    in.failsafe_active = false;
    in.altitude_hold_enabled = false;
    in.critical_fault = false;
    return in;
}

static void test_boot_always_advances_to_disarmed(void)
{
    flight_mode_inputs_t in = base_inputs();
    in.arm_command = false;
    in.arming_preconditions_met = false;
    in.critical_fault = true; // even a fault-asserting cycle - BOOT never checks it (see flight_mode.c)
    CHECK(flight_mode_transition(FLIGHT_MODE_BOOT, &in) == FLIGHT_MODE_DISARMED,
          "BOOT -> DISARMED unconditionally");
}

static void test_disarmed_stays_without_arm_command(void)
{
    flight_mode_inputs_t in = base_inputs();
    in.arm_command = false;
    CHECK(flight_mode_transition(FLIGHT_MODE_DISARMED, &in) == FLIGHT_MODE_DISARMED,
          "DISARMED stays DISARMED with arm_command=false");
}

static void test_disarmed_stays_when_preconditions_not_met(void)
{
    flight_mode_inputs_t in = base_inputs();
    in.arm_command = true;
    in.arming_preconditions_met = false;
    CHECK(flight_mode_transition(FLIGHT_MODE_DISARMED, &in) == FLIGHT_MODE_DISARMED,
          "DISARMED stays DISARMED when preconditions not met, even with arm_command=true");
}

static void test_disarmed_to_armed(void)
{
    flight_mode_inputs_t in = base_inputs();
    in.arm_command = true;
    in.arming_preconditions_met = true;
    CHECK(flight_mode_transition(FLIGHT_MODE_DISARMED, &in) == FLIGHT_MODE_ARMED,
          "DISARMED -> ARMED when arm_command && preconditions_met");
}

static void test_disarmed_never_reaches_failsafe_directly(void)
{
    // flight_mode_t has no DISARMED -> FAILSAFE edge at all: failsafe_active is simply not
    // consulted by the DISARMED branch (see flight_mode.c) - a failsafe-worthy condition while
    // disarmed is instead expected to surface through arming_preconditions_met() being false
    // (arming.h's no_critical_errors field), blocking DISARMED -> ARMED instead. This test
    // confirms the state machine itself has no code path from DISARMED to FAILSAFE, for any
    // combination of arm_command/failsafe_active.
    flight_mode_t possible_next[] = {FLIGHT_MODE_DISARMED, FLIGHT_MODE_ARMED, FLIGHT_MODE_ERROR};
    bool arm_values[] = {false, true};
    bool failsafe_values[] = {false, true};
    bool preconditions_values[] = {false, true};

    for (size_t a = 0; a < 2; a++) {
        for (size_t f = 0; f < 2; f++) {
            for (size_t p = 0; p < 2; p++) {
                flight_mode_inputs_t in = base_inputs();
                in.arm_command = arm_values[a];
                in.failsafe_active = failsafe_values[f];
                in.arming_preconditions_met = preconditions_values[p];

                flight_mode_t next = flight_mode_transition(FLIGHT_MODE_DISARMED, &in);
                bool is_one_of_expected = false;
                for (size_t i = 0; i < sizeof(possible_next) / sizeof(possible_next[0]); i++) {
                    if (next == possible_next[i]) {
                        is_one_of_expected = true;
                    }
                }
                CHECK(is_one_of_expected && next != FLIGHT_MODE_FAILSAFE,
                      "DISARMED never transitions to FAILSAFE, for any arm/failsafe/"
                      "preconditions input combination");
            }
        }
    }
}

static void test_disarmed_to_error_and_back(void)
{
    flight_mode_inputs_t in = base_inputs();
    in.critical_fault = true;
    CHECK(flight_mode_transition(FLIGHT_MODE_DISARMED, &in) == FLIGHT_MODE_ERROR,
          "DISARMED -> ERROR when critical_fault is asserted");

    // ERROR ignores arm_command entirely (motors are never live in ERROR) - it only clears when
    // critical_fault clears.
    in.arm_command = false;
    CHECK(flight_mode_transition(FLIGHT_MODE_ERROR, &in) == FLIGHT_MODE_ERROR,
          "ERROR stays ERROR while critical_fault is still true, regardless of arm_command");

    in.critical_fault = false;
    CHECK(flight_mode_transition(FLIGHT_MODE_ERROR, &in) == FLIGHT_MODE_DISARMED,
          "ERROR -> DISARMED once critical_fault clears");
}

static void test_armed_mode_requests(void)
{
    flight_mode_inputs_t in = base_inputs();

    in.mode_request = FLIGHT_MODE_REQUEST_STABILIZE;
    CHECK(flight_mode_transition(FLIGHT_MODE_ARMED, &in) == FLIGHT_MODE_STABILIZE,
          "ARMED -> STABILIZE on request");

    in.mode_request = FLIGHT_MODE_REQUEST_ALTITUDE_HOLD;
    in.altitude_hold_enabled = false;
    CHECK(flight_mode_transition(FLIGHT_MODE_ARMED, &in) == FLIGHT_MODE_ARMED,
          "ARMED stays ARMED when ALTITUDE_HOLD is requested but disabled (no-op)");

    in.altitude_hold_enabled = true;
    CHECK(flight_mode_transition(FLIGHT_MODE_ARMED, &in) == FLIGHT_MODE_ALTITUDE_HOLD,
          "ARMED -> ALTITUDE_HOLD on request when enabled");

    in.mode_request = FLIGHT_MODE_REQUEST_ARMED_IDLE;
    CHECK(flight_mode_transition(FLIGHT_MODE_ARMED, &in) == FLIGHT_MODE_ARMED,
          "ARMED stays ARMED on an idle request");
}

static void test_stabilize_and_altitude_hold_transitions(void)
{
    flight_mode_inputs_t in = base_inputs();
    in.altitude_hold_enabled = true;

    in.mode_request = FLIGHT_MODE_REQUEST_ARMED_IDLE;
    CHECK(flight_mode_transition(FLIGHT_MODE_STABILIZE, &in) == FLIGHT_MODE_ARMED,
          "STABILIZE -> ARMED on idle request");

    in.mode_request = FLIGHT_MODE_REQUEST_ALTITUDE_HOLD;
    CHECK(flight_mode_transition(FLIGHT_MODE_STABILIZE, &in) == FLIGHT_MODE_ALTITUDE_HOLD,
          "STABILIZE -> ALTITUDE_HOLD on request when enabled");

    in.mode_request = FLIGHT_MODE_REQUEST_STABILIZE;
    CHECK(flight_mode_transition(FLIGHT_MODE_ALTITUDE_HOLD, &in) == FLIGHT_MODE_STABILIZE,
          "ALTITUDE_HOLD -> STABILIZE on request");

    in.mode_request = FLIGHT_MODE_REQUEST_ARMED_IDLE;
    CHECK(flight_mode_transition(FLIGHT_MODE_ALTITUDE_HOLD, &in) == FLIGHT_MODE_ARMED,
          "ALTITUDE_HOLD -> ARMED on idle request");
}

static void test_altitude_hold_gated_off_is_a_noop_from_every_flying_state(void)
{
    flight_mode_t sources[] = {FLIGHT_MODE_ARMED, FLIGHT_MODE_STABILIZE};
    for (size_t i = 0; i < sizeof(sources) / sizeof(sources[0]); i++) {
        flight_mode_inputs_t in = base_inputs();
        in.altitude_hold_enabled = false;
        in.mode_request = FLIGHT_MODE_REQUEST_ALTITUDE_HOLD;
        CHECK(flight_mode_transition(sources[i], &in) == sources[i],
              "ALTITUDE_HOLD request while gated off is a no-op (stays in current mode)");
    }
}

static void test_failsafe_reachable_from_every_flying_state(void)
{
    flight_mode_t sources[] = {FLIGHT_MODE_ARMED, FLIGHT_MODE_STABILIZE, FLIGHT_MODE_ALTITUDE_HOLD};
    for (size_t i = 0; i < sizeof(sources) / sizeof(sources[0]); i++) {
        flight_mode_inputs_t in = base_inputs();
        in.failsafe_active = true;
        CHECK(flight_mode_transition(sources[i], &in) == FLIGHT_MODE_FAILSAFE,
              "an active failsafe condition forces FAILSAFE from every flying state");
    }
}

static void test_failsafe_is_latching_until_explicit_disarm(void)
{
    flight_mode_inputs_t in = base_inputs();
    in.failsafe_active = true;
    CHECK(flight_mode_transition(FLIGHT_MODE_STABILIZE, &in) == FLIGHT_MODE_FAILSAFE,
          "STABILIZE -> FAILSAFE");

    // Condition clears, but arm_command was never toggled - FAILSAFE must not auto-resume flight.
    in.failsafe_active = false;
    CHECK(flight_mode_transition(FLIGHT_MODE_FAILSAFE, &in) == FLIGHT_MODE_FAILSAFE,
          "FAILSAFE does not auto-clear just because the condition that caused it clears - "
          "explicit disarm is required (never auto-arm/auto-resume)");

    // The only way out: explicit disarm.
    in.arm_command = false;
    CHECK(flight_mode_transition(FLIGHT_MODE_FAILSAFE, &in) == FLIGHT_MODE_DISARMED,
          "FAILSAFE -> DISARMED via explicit disarm command");
}

static void test_fast_disarm_from_every_live_state(void)
{
    flight_mode_t sources[] = {FLIGHT_MODE_ARMED, FLIGHT_MODE_STABILIZE, FLIGHT_MODE_ALTITUDE_HOLD,
                                FLIGHT_MODE_FAILSAFE};
    for (size_t i = 0; i < sizeof(sources) / sizeof(sources[0]); i++) {
        flight_mode_inputs_t in = base_inputs();
        in.arm_command = false;
        // Even with a failsafe still nominally active, disarm wins immediately.
        in.failsafe_active = (sources[i] == FLIGHT_MODE_FAILSAFE);
        CHECK(flight_mode_transition(sources[i], &in) == FLIGHT_MODE_DISARMED,
              "dropping arm_command returns to DISARMED in one cycle from any live state");
    }
}

static void test_disarm_then_rearm_requires_fresh_preconditions(void)
{
    // Enter FAILSAFE, disarm, then attempt to re-arm while a precondition is still failing -
    // must NOT auto-arm.
    flight_mode_inputs_t in = base_inputs();
    in.failsafe_active = true;
    flight_mode_t mode = flight_mode_transition(FLIGHT_MODE_STABILIZE, &in);
    CHECK(mode == FLIGHT_MODE_FAILSAFE, "setup: entered FAILSAFE");

    in.arm_command = false;
    mode = flight_mode_transition(mode, &in);
    CHECK(mode == FLIGHT_MODE_DISARMED, "setup: disarmed from FAILSAFE");

    in.arm_command = true;
    in.failsafe_active = false;
    in.arming_preconditions_met = false; // e.g. radio not yet reacquired
    mode = flight_mode_transition(mode, &in);
    CHECK(mode == FLIGHT_MODE_DISARMED,
          "re-arm attempt with a still-failing precondition stays DISARMED, never auto-arms");

    in.arming_preconditions_met = true;
    mode = flight_mode_transition(mode, &in);
    CHECK(mode == FLIGHT_MODE_ARMED, "re-arm succeeds once every precondition is met again");
}

static void test_is_armed(void)
{
    CHECK(!flight_mode_is_armed(FLIGHT_MODE_BOOT), "BOOT is not armed");
    CHECK(!flight_mode_is_armed(FLIGHT_MODE_DISARMED), "DISARMED is not armed");
    CHECK(flight_mode_is_armed(FLIGHT_MODE_ARMED), "ARMED is armed");
    CHECK(flight_mode_is_armed(FLIGHT_MODE_STABILIZE), "STABILIZE is armed");
    CHECK(flight_mode_is_armed(FLIGHT_MODE_ALTITUDE_HOLD), "ALTITUDE_HOLD is armed");
    CHECK(!flight_mode_is_armed(FLIGHT_MODE_FAILSAFE), "FAILSAFE is not armed");
    CHECK(!flight_mode_is_armed(FLIGHT_MODE_ERROR), "ERROR is not armed");
}

static void test_name_never_null(void)
{
    flight_mode_t modes[] = {FLIGHT_MODE_BOOT,     FLIGHT_MODE_DISARMED, FLIGHT_MODE_ARMED,
                              FLIGHT_MODE_STABILIZE, FLIGHT_MODE_ALTITUDE_HOLD,
                              FLIGHT_MODE_FAILSAFE, FLIGHT_MODE_ERROR};
    for (size_t i = 0; i < sizeof(modes) / sizeof(modes[0]); i++) {
        CHECK(flight_mode_name(modes[i]) != NULL, "flight_mode_name never returns NULL");
    }
}

static void test_null_input_is_a_noop(void)
{
    CHECK(flight_mode_transition(FLIGHT_MODE_ARMED, NULL) == FLIGHT_MODE_ARMED,
          "a NULL inputs pointer never crashes and never changes mode");
}

int main(void)
{
    test_boot_always_advances_to_disarmed();
    test_disarmed_stays_without_arm_command();
    test_disarmed_stays_when_preconditions_not_met();
    test_disarmed_to_armed();
    test_disarmed_never_reaches_failsafe_directly();
    test_disarmed_to_error_and_back();
    test_armed_mode_requests();
    test_stabilize_and_altitude_hold_transitions();
    test_altitude_hold_gated_off_is_a_noop_from_every_flying_state();
    test_failsafe_reachable_from_every_flying_state();
    test_failsafe_is_latching_until_explicit_disarm();
    test_fast_disarm_from_every_live_state();
    test_disarm_then_rearm_requires_fresh_preconditions();
    test_is_armed();
    test_name_never_null();
    test_null_input_is_a_noop();

    printf("%d/%d checks passed\n", g_checks - g_failures, g_checks);
    return g_failures == 0 ? 0 : 1;
}
