// The one piece of genuinely cross-task shared state: the vehicle's current flight mode plus
// diagnostics about why. SafetyTask (as of Milestone 16, real transition logic - see
// flight_mode.h/arming.h/failsafe.h and docs/safety.md) is the sole writer; FlightControlTask and
// TelemetryTask are readers. This is a multi-field struct that must be observed consistently (a
// reader must never see `mode` from one write interleaved with `checked_at_us` from another), so
// it is protected by a mutex - see safety_state.c for why this is the only spot in the milestone
// that needs one, unlike the sensor queue (single writer/reader, queue already serializes it) or
// the notification counts (single word, atomic by construction).
//
// Milestone 6 shipped this as `{ armed, checked_at_us }` with `armed` always false (no real
// arming logic existed yet). Milestone 16 extends it in place, rather than replacing it, so
// existing readers (flight_control_task.c, telemetry_task.c) keep working with `armed` meaning
// exactly what it always meant (flight_mode_is_armed(mode)) while gaining access to the real
// mode/diagnostics.
#ifndef BICOPTER_MAIN_SAFETY_STATE_H
#define BICOPTER_MAIN_SAFETY_STATE_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#include "failsafe.h"
#include "flight_mode.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool armed;                              // flight_mode_is_armed(mode) - kept as a separate
                                               // field (not just derived at each read site) so
                                               // existing callers from Milestone 6 need no changes
    flight_mode_t mode;
    failsafe_reason_t last_failsafe_reason;   // the reason that most recently drove mode into
                                               // FAILSAFE; FAILSAFE_REASON_NONE if never triggered
                                               // this boot. Persists across the FAILSAFE -> DISARMED
                                               // transition (for post-flight diagnosis/telemetry)
                                               // until the next arm attempt.
    uint32_t arming_blocking_mask;            // arming_blocking_mask() (arming.h) result from the
                                               // most recent cycle - which precondition(s), if any,
                                               // are currently blocking DISARMED -> ARMED. Zero
                                               // whenever mode != DISARMED (nothing to report).
    int64_t checked_at_us;                    // esp_timer_get_time() timestamp of SafetyTask's
                                               // last write
} safety_state_t;

// Creates the backing mutex. Must be called once from main() before any task that touches safety
// state is started.
esp_err_t safety_state_init(void);

// Writer: SafetyTask only. Takes the mutex, updates every field together, releases it.
void safety_state_set(const safety_state_t *state);

// Reader: FlightControlTask, TelemetryTask. Takes the mutex, copies every field out, releases it.
void safety_state_get(safety_state_t *out_state);

#ifdef __cplusplus
}
#endif

#endif // BICOPTER_MAIN_SAFETY_STATE_H
