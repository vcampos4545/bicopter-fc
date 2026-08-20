// The one piece of this milestone's task logic that is pure enough to be worth pulling out and
// host-testing independently, per AGENTS.md's "Driver testing convention": SensorTask's
// baro-read decimation check. Everything else in main/'s tasks is FreeRTOS/ESP-IDF scaffolding
// (task loops, queues, notifications, a mutex) that isn't meaningfully testable off-target - see
// tests/task_util_test.c and the milestone 6 report in docs/architecture.md for why this is the
// only new host test this milestone adds.
#ifndef BICOPTER_MAIN_TASK_UTIL_H
#define BICOPTER_MAIN_TASK_UTIL_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// True on the cycle SensorTask should re-read the barometer: `cycle_count` is a 1-based loop
// counter, `decimation` is how many sensor cycles make up one barometer cycle (see
// SENSOR_TASK_BARO_DECIMATION in task_config.h). A `decimation` of 0 or 1 means "every cycle";
// there is no divide-by-zero case since 0 is treated the same as 1.
bool task_should_decimate(uint32_t cycle_count, uint32_t decimation);

#ifdef __cplusplus
}
#endif

#endif // BICOPTER_MAIN_TASK_UTIL_H
