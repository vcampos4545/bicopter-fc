#ifndef BICOPTER_MAIN_SENSOR_TASK_H
#define BICOPTER_MAIN_SENSOR_TASK_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "barometer.h"
#include "imu.h"

#ifdef __cplusplus
extern "C" {
#endif

// One handoff unit from SensorTask to EstimatorTask: the latest IMU sample (read every cycle)
// paired with the latest barometer sample (read only every SENSOR_TASK_BARO_DECIMATION cycles -
// carried forward unchanged on cycles it wasn't re-read; its own `timestamp_us`/`valid` fields
// tell a consumer how fresh it is).
typedef struct {
    imu_reading_t imu;
    barometer_reading_t baro;
} sensor_sample_t;

typedef struct {
    // Length-1 "latest sample" mailbox, written every cycle via xQueueOverwrite(). SensorTask is
    // this milestone's highest-priority periodic task short of SafetyTask and must never block on
    // a slow downstream reader, so this is overwrite semantics rather than a blocking xQueueSend -
    // EstimatorTask always gets the most recent sample, never a backlog of stale ones.
    QueueHandle_t sample_queue;
} sensor_task_params_t;

void sensor_task(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif // BICOPTER_MAIN_SENSOR_TASK_H
