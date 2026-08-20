#ifndef BICOPTER_MAIN_SAFETY_TASK_H
#define BICOPTER_MAIN_SAFETY_TASK_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    // FlightControlTask's handle. SafetyTask calls xTaskNotifyGive() on it after every write to
    // the mutex-protected safety_state_t (see safety_state.h) - a lightweight "state changed, go
    // look" wake-up that is deliberately separate from the state itself: the notification is a
    // single word (no payload needed, so a notification is the right tool, not a queue), while
    // the actual multi-field state needs the mutex because a reader must never observe one field
    // updated and the other stale (a torn read across `armed` and `checked_at_us`).
    TaskHandle_t flight_control_handle;
} safety_task_params_t;

void safety_task(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif // BICOPTER_MAIN_SAFETY_TASK_H
