// Flight-mode state machine: BOOT -> DISARMED -> ARMED -> {STABILIZE, ALTITUDE_HOLD} -> FAILSAFE
// -> ERROR, per docs/safety.md. Pure, ESP-IDF-free logic (same driver-testing-convention split as
// firmware/components/sensors/include/mpu6050_convert.h) so the full transition table is
// host-tested (see tests/flight_mode_test.c) rather than only reviewed by eye.
//
// This lives in firmware/components/safety/, not flight_core/safety/, deliberately: flight_core
// is a C++17 library not yet wired into firmware as an ESP-IDF component (the same open item
// AGENTS.md already notes for EstimatorTask/FlightControlTask since Milestones 8/10-13). Routing
// this milestone's must-be-wired-into-SafetyTask-today logic through that not-yet-existing bridge
// would either block this milestone on unrelated infra work or leave it as unwired as
// flight_core/dynamics/ has been since Milestone 1 - see docs/safety.md's "Why firmware/, not
// flight_core/" section for the full reasoning. flight_core/safety/ remains an empty scaffold.
#ifndef BICOPTER_SAFETY_FLIGHT_MODE_H
#define BICOPTER_SAFETY_FLIGHT_MODE_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    FLIGHT_MODE_BOOT = 0,
    FLIGHT_MODE_DISARMED,
    FLIGHT_MODE_ARMED,
    FLIGHT_MODE_STABILIZE,
    FLIGHT_MODE_ALTITUDE_HOLD,
    FLIGHT_MODE_FAILSAFE,
    FLIGHT_MODE_ERROR,
} flight_mode_t;

// Decoded from the radio's raw flight_mode index (radio_command_t.flight_mode) by the caller -
// this header stays free of any radio-specific mapping. REQUEST_ARMED_IDLE is the "flying modes
// not requested" value (radio_command_t.flight_mode == 0), the switch position that keeps an
// armed vehicle at idle rather than commanding either flying mode.
typedef enum {
    FLIGHT_MODE_REQUEST_ARMED_IDLE = 0,
    FLIGHT_MODE_REQUEST_STABILIZE,
    FLIGHT_MODE_REQUEST_ALTITUDE_HOLD,
} flight_mode_request_t;

typedef struct {
    // Pilot arm switch/command (radio_command_t.arm). Level-triggered: false forces DISARMED from
    // any state within one cycle - see docs/safety.md's "disarm is always fast" rule.
    bool arm_command;

    // Decoded flying-mode request; only consulted while already armed (ARMED/STABILIZE/
    // ALTITUDE_HOLD) and no failsafe is active.
    flight_mode_request_t mode_request;

    // Result of arming_preconditions_met() (arming.h) for the current cycle. Only consulted from
    // DISARMED - STABILIZE/ALTITUDE_HOLD do not re-check arming preconditions on every cycle
    // (that would risk mid-flight demotion on a transient blip); losing a precondition mid-flight
    // shows up as a failsafe condition instead (see failsafe.h), which routes to FAILSAFE, not
    // DISARMED.
    bool arming_preconditions_met;

    // Result of (failsafe_evaluate() != FAILSAFE_REASON_NONE) for the current cycle. Only
    // consulted from ARMED/STABILIZE/ALTITUDE_HOLD/FAILSAFE - see docs/safety.md for why FAILSAFE
    // is not reachable from DISARMED/BOOT (nothing to fail safe from when motors are already idle;
    // a failsafe-worthy condition present while disarmed instead surfaces through
    // arming_preconditions_met() being false, blocking DISARMED -> ARMED).
    bool failsafe_active;

    // Kconfig gate (CONFIG_BICOPTER_ALTITUDE_HOLD_ENABLED, default off - see docs/safety.md).
    // Passed in rather than read from Kconfig here so this whole module stays host-testable
    // without any ESP-IDF/sdkconfig dependency.
    bool altitude_hold_enabled;

    // Reserved hook for a future non-in-flight fault detector (e.g. a config-validation failure
    // at boot) to route BOOT/DISARMED into ERROR. No caller in this milestone's firmware sets this
    // true - see docs/safety.md's "ERROR is ground-only" section for why in-flight faults always
    // route through failsafe_active instead, and why that is a deliberate single-path design, not
    // a gap. Kept here (rather than omitted) so the transition table and its tests are already
    // correct and exercised for whenever a real detector lands.
    bool critical_fault;
} flight_mode_inputs_t;

// Pure state-transition function: given the current mode and this cycle's inputs, returns the
// next mode. Deterministic, total (never asserts/aborts on any input combination), and evaluated
// in a fixed, documented priority order - see flight_mode.c and docs/safety.md's transition table
// for the exact precedence (fast-disarm first, then failsafe, then normal progression).
flight_mode_t flight_mode_transition(flight_mode_t current, const flight_mode_inputs_t *in);

// True for the three modes in which motors may be commanded above idle (ARMED counts as armed-but-
// idle: motors are enabled but not yet under stabilization output). Convenience for callers that
// only care about "is this mode's motor output potentially live," e.g. logging/telemetry.
bool flight_mode_is_armed(flight_mode_t mode);

// Human-readable name, for logging/telemetry - never NULL.
const char *flight_mode_name(flight_mode_t mode);

#ifdef __cplusplus
}
#endif

#endif // BICOPTER_SAFETY_FLIGHT_MODE_H
