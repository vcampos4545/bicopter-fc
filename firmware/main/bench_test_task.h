// Milestone 18 (see ../../docs/bench_test.md): BenchTestTask owns the bench-test console (UART
// command interface) and dispatches each parsed command against real firmware state - the sensor
// sample queue (peeked, same pattern safety_task.c already uses), radio_state.h, and the
// actuators_t published via actuators_state.h. Each command family is independently usable per
// docs/bench_test.md's requirement: sensor/radio streaming only ever read existing state, servo
// only needs actuators enabled, and esc_test additionally needs a HARDWARE_TEST build.
//
// Deliberately not registered with the task watchdog (unlike this project's six core tasks - see
// task_config.h) and not given a fixed vTaskDelayUntil period: it is an interactive diagnostic
// tool, not part of the real-time flight-control path, and its esc_test handler intentionally
// blocks for CONFIG_BICOPTER_BENCH_TEST_ESC_TEST_DURATION_MS while a motor test runs (see
// bench_test_task.c) - a duration a strict watchdog period would fight, not help.
#ifndef BICOPTER_MAIN_BENCH_TEST_TASK_H
#define BICOPTER_MAIN_BENCH_TEST_TASK_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    // SensorTask's sample queue (sensor_task.h) - peeked, never consumed, same reasoning
    // safety_task.h documents for its own copy of this handle.
    QueueHandle_t sensor_sample_queue;
} bench_test_task_params_t;

void bench_test_task(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif // BICOPTER_MAIN_BENCH_TEST_TASK_H
