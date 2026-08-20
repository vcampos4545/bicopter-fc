// The one piece of genuinely cross-task shared state in this milestone's scaffolding: the
// vehicle's armed/disarmed status plus the timestamp of SafetyTask's last check. SafetyTask
// (milestone 16 will give it real failsafe conditions) is the sole writer; FlightControlTask and
// TelemetryTask are readers. This is a multi-field struct that must be observed consistently (a
// reader must never see `armed` from one write interleaved with `checked_at_us` from another), so
// it is protected by a mutex - see safety_state.c for why this is the only spot in the milestone
// that needs one, unlike the sensor queue (single writer/reader, queue already serializes it) or
// the notification counts (single word, atomic by construction).
#ifndef BICOPTER_MAIN_SAFETY_STATE_H
#define BICOPTER_MAIN_SAFETY_STATE_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool armed;               // always false in this milestone - no real arming logic exists yet
                               // (that's milestone 16); the field and its plumbing exist now so
                               // later milestones extend, not invent, this path.
    int64_t checked_at_us;    // esp_timer_get_time() timestamp of SafetyTask's last write
} safety_state_t;

// Creates the backing mutex. Must be called once from main() before any task that touches safety
// state is started.
esp_err_t safety_state_init(void);

// Writer: SafetyTask only. Takes the mutex, updates both fields together, releases it.
void safety_state_set(bool armed, int64_t checked_at_us);

// Reader: FlightControlTask, TelemetryTask. Takes the mutex, copies both fields out, releases it.
void safety_state_get(safety_state_t *out_state);

#ifdef __cplusplus
}
#endif

#endif // BICOPTER_MAIN_SAFETY_STATE_H
