#ifndef BICOPTER_MAIN_SAFETY_TASK_H
#define BICOPTER_MAIN_SAFETY_TASK_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
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
    // updated and the other stale (a torn read across fields).
    TaskHandle_t flight_control_handle;

    // SensorTask's sample queue (sensor_task.h). SafetyTask never consumes from this queue
    // (xQueueReceive would race EstimatorTask for the one item) - it only xQueuePeek()s the
    // latest sample each cycle, which is safe for any number of concurrent readers and doesn't
    // disturb EstimatorTask's own consumption. See safety_task.c for why a peek, not a second
    // mutex-protected publish like radio_state.h's, is the right tool here: the queue already
    // serializes SensorTask's writes, and peeking adds no new synchronization primitive.
    QueueHandle_t sensor_sample_queue;
} safety_task_params_t;

void safety_task(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif // BICOPTER_MAIN_SAFETY_TASK_H
