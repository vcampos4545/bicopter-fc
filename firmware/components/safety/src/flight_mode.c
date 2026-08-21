#include "flight_mode.h"

#include <stddef.h>

flight_mode_t flight_mode_transition(flight_mode_t current, const flight_mode_inputs_t *in)
{
    if (in == NULL) {
        return current;
    }

    // 1. BOOT always advances to DISARMED. No boot self-test exists yet beyond task/mutex/driver
    // init succeeding, which happens before SafetyTask's first cycle (and therefore before this
    // function is ever called) - reaching here at all already means boot succeeded. A future
    // real boot self-test would set critical_fault instead of adding a new BOOT branch here.
    if (current == FLIGHT_MODE_BOOT) {
        return FLIGHT_MODE_DISARMED;
    }

    // 2. ERROR only clears when the fault that caused it clears - never via arm_command. Motors
    // are never live in ERROR, so there is no "fast disarm" to apply here.
    if (current == FLIGHT_MODE_ERROR) {
        return in->critical_fault ? FLIGHT_MODE_ERROR : FLIGHT_MODE_DISARMED;
    }

    // 3. Fast disarm: dropping the arm command returns to DISARMED from any live state in one
    // cycle, including FAILSAFE - this is deliberately unconditional (no precondition/failsafe
    // check gates it) per docs/safety.md's "disarm is always fast" rule, and it is the *only* way
    // out of FAILSAFE (FAILSAFE is otherwise latching - see below).
    if (!in->arm_command && current != FLIGHT_MODE_DISARMED) {
        return FLIGHT_MODE_DISARMED;
    }

    if (current == FLIGHT_MODE_DISARMED) {
        if (in->critical_fault) {
            return FLIGHT_MODE_ERROR;
        }
        if (in->arm_command && in->arming_preconditions_met) {
            return FLIGHT_MODE_ARMED;
        }
        return FLIGHT_MODE_DISARMED;
    }

    // From here, current is ARMED/STABILIZE/ALTITUDE_HOLD/FAILSAFE and arm_command is true (an
    // arm_command of false would already have returned via the fast-disarm branch above).

    // 4. Any active failsafe condition wins over mode-request handling - checked every cycle, not
    // just on entry, so a still-active condition keeps re-asserting FAILSAFE even if arm_command
    // stays true (this is what makes FAILSAFE latching: the only way out is the fast-disarm path).
    if (in->failsafe_active) {
        return FLIGHT_MODE_FAILSAFE;
    }
    if (current == FLIGHT_MODE_FAILSAFE) {
        // failsafe_active reads false this cycle but the caller never re-runs arming
        // preconditions here (see flight_mode_inputs_t.mode_request doc) - by design, clearing a
        // failsafe condition does not auto-resume flight. Explicit disarm + re-arm is required,
        // which is the fast-disarm branch above followed by the normal DISARMED -> ARMED path
        // (itself gated on arming_preconditions_met()).
        return FLIGHT_MODE_FAILSAFE;
    }

    // 5. Normal in-flight mode-request handling (ARMED/STABILIZE/ALTITUDE_HOLD only).
    switch (in->mode_request) {
    case FLIGHT_MODE_REQUEST_ARMED_IDLE:
        return FLIGHT_MODE_ARMED;
    case FLIGHT_MODE_REQUEST_STABILIZE:
        return FLIGHT_MODE_STABILIZE;
    case FLIGHT_MODE_REQUEST_ALTITUDE_HOLD:
        // Requesting the experimental mode while its Kconfig gate is off is a no-op: the state
        // machine stays in whatever mode it was already in, rather than guessing whether the
        // pilot would prefer idle or STABILIZE instead (see flight_mode.h).
        return in->altitude_hold_enabled ? FLIGHT_MODE_ALTITUDE_HOLD : current;
    default:
        return current;
    }
}

bool flight_mode_is_armed(flight_mode_t mode)
{
    return mode == FLIGHT_MODE_ARMED || mode == FLIGHT_MODE_STABILIZE ||
           mode == FLIGHT_MODE_ALTITUDE_HOLD;
}

const char *flight_mode_name(flight_mode_t mode)
{
    switch (mode) {
    case FLIGHT_MODE_BOOT:
        return "BOOT";
    case FLIGHT_MODE_DISARMED:
        return "DISARMED";
    case FLIGHT_MODE_ARMED:
        return "ARMED";
    case FLIGHT_MODE_STABILIZE:
        return "STABILIZE";
    case FLIGHT_MODE_ALTITUDE_HOLD:
        return "ALTITUDE_HOLD";
    case FLIGHT_MODE_FAILSAFE:
        return "FAILSAFE";
    case FLIGHT_MODE_ERROR:
        return "ERROR";
    default:
        return "UNKNOWN";
    }
}
