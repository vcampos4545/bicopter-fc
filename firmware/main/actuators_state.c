#include "actuators_state.h"

#include <string.h>

static actuators_t s_actuators;
static bool s_ready;

void actuators_state_set(const actuators_t *actuators, bool ready)
{
    s_ready = ready;
    if (ready && actuators != NULL) {
        s_actuators = *actuators;
    } else {
        memset(&s_actuators, 0, sizeof(s_actuators));
    }
}

bool actuators_state_get(actuators_t *out_actuators)
{
    if (!s_ready || out_actuators == NULL) {
        return false;
    }
    *out_actuators = s_actuators;
    return true;
}
