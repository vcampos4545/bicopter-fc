#ifndef BICOPTER_MAIN_ESTIMATOR_TASK_H
#define BICOPTER_MAIN_ESTIMATOR_TASK_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    QueueHandle_t sample_queue; // same queue SensorTask publishes to (see sensor_task.h)
} estimator_task_params_t;

void estimator_task(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif // BICOPTER_MAIN_ESTIMATOR_TASK_H
