// Milestone 16 addition: publishes RadioTask's latest decoded command + link health so SafetyTask
// can evaluate real arming/failsafe preconditions against them. Neither radio_command_t nor
// radio_health_t existed as cross-task state before this milestone - RadioTask (Milestone 14/15)
// only ever logged them locally, since nothing downstream consumed a live setpoint yet (see
// AGENTS.md: "Does not yet deposit setpoints into FlightControlTask - that wiring is a distinct,
// still-unstarted piece of work"). This module is that missing publish step, scoped to exactly
// what SafetyTask needs (arm command, flight-mode request, link health) - it is not the general
// RadioTask -> FlightControlTask setpoint plumbing that AGENTS.md notes is still unstarted.
//
// Same mutex-protected read/write shape as safety_state.h, for the same reason: radio_state_t is
// a multi-field struct that must be observed consistently.
#ifndef BICOPTER_MAIN_RADIO_STATE_H
#define BICOPTER_MAIN_RADIO_STATE_H

#include <stdbool.h>

#include "esp_err.h"

#include "radio.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool available;            // true iff a radio transport is enabled and initialized this boot
                                // (CONFIG_BICOPTER_RADIO_ENABLED or _CRSF_RADIO_ENABLED) - false
                                // whenever no transport exists, same as radio_task.c's own
                                // `radio_ready` local. A caller must treat unavailable exactly like
                                // "link not alive," never like "no opinion."
    bool has_command;          // radio_has_command() result from the most recent RadioTask cycle
    radio_command_t command;   // most recently accepted command (only meaningful if has_command)
    radio_health_t health;     // most recent radio_get_health() result
} radio_state_t;

// Creates the backing mutex. Must be called once from main() before any task that touches radio
// state is started.
esp_err_t radio_state_init(void);

// Writer: RadioTask only.
void radio_state_set(const radio_state_t *state);

// Reader: SafetyTask (and, later, FlightControlTask once real setpoint consumption lands).
void radio_state_get(radio_state_t *out_state);

#ifdef __cplusplus
}
#endif

#endif // BICOPTER_MAIN_RADIO_STATE_H
