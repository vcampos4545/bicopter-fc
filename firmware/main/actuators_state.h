// Milestone 18 (see ../../docs/bench_test.md): publishes the actuators_t main() builds when
// CONFIG_BICOPTER_ACTUATORS_ENABLED is set, so BenchTestTask's servo/esc_test console commands can
// command real actuator handles directly - independent of (and much narrower than) the real
// flight-control actuator path FlightControlTask still doesn't call (see AGENTS.md).
//
// Unlike safety_state.h/radio_state.h (both written every cycle by their owning task, and so
// mutex-protected against torn multi-field reads), this needs no mutex: main() calls
// actuators_state_set() exactly once, before any task that might read it is created, and never
// again - a value written once before concurrent access begins and read-only afterward has no
// torn-read window to protect.
#ifndef BICOPTER_MAIN_ACTUATORS_STATE_H
#define BICOPTER_MAIN_ACTUATORS_STATE_H

#include <stdbool.h>

#include "actuators_init.h"

#ifdef __cplusplus
extern "C" {
#endif

// Writer: main() only, before any task is created.
void actuators_state_set(const actuators_t *actuators, bool ready);

// Returns true and fills *out_actuators iff CONFIG_BICOPTER_ACTUATORS_ENABLED was set and
// actuators_init_safe() succeeded this boot. Returns false (out_actuators untouched) otherwise -
// callers (BenchTestTask's servo/esc_test handlers) must treat that as "no actuators to command,"
// never guess or attempt to use a zeroed actuators_t.
bool actuators_state_get(actuators_t *out_actuators);

#ifdef __cplusplus
}
#endif

#endif // BICOPTER_MAIN_ACTUATORS_STATE_H
