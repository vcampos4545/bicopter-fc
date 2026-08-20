// Central FreeRTOS priority/period/stack configuration for Milestone 6's task architecture.
// Single source of truth so docs/architecture.md's task table and this file can't silently drift
// apart - change a number here, update the table (and the reasoning, if it changed) in the same
// commit.
//
// Priority scale: FreeRTOS priorities are integers from 0 (tskIDLE_PRIORITY, lowest) up to
// configMAX_PRIORITIES-1 (24 in this project's config). ESP-IDF's own system tasks (the I2C
// driver's internal task, the esp_timer dispatch task at ESP_TASK_TIMER_PRIO =
// configMAX_PRIORITIES-3 = 22, Wi-Fi/lwIP once radio exists) run at high priorities, so this
// project's six tasks are placed in the low-to-mid range, below all of that, and ranked relative
// to each other by *consequence of a missed deadline* per docs/architecture.md's FreeRTOS task
// table - not simply by data rate:
//
//   SafetyTask (10) > SensorTask (9) > EstimatorTask (8) > FlightControlTask (7)
//     > RadioTask (5) > TelemetryTask (3)
//
// SafetyTask outranks everything because a late failsafe is a crash risk. SensorTask/
// EstimatorTask sit just below it and adjacent to each other because IMU gyro-integration error
// grows with sample latency/jitter and the estimate must reflect a new sample with minimal delay
// (see estimator_task.c). FlightControlTask trails them because bicopter actuator response (ESC
// spin-up, servo slew) tolerates more jitter than the estimate itself does. RadioTask and
// TelemetryTask are deliberately low: a slow/stalled radio link or telemetry downlink must never
// win CPU contention against the control path (see docs/architecture.md's "General scheduling
// principles").
#ifndef BICOPTER_MAIN_TASK_CONFIG_H
#define BICOPTER_MAIN_TASK_CONFIG_H

#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BICOPTER_TASK_STACK_SIZE_BYTES 4096

#define SAFETY_TASK_PRIORITY         (tskIDLE_PRIORITY + 10)
#define SENSOR_TASK_PRIORITY         (tskIDLE_PRIORITY + 9)
#define ESTIMATOR_TASK_PRIORITY      (tskIDLE_PRIORITY + 8)
#define FLIGHT_CONTROL_TASK_PRIORITY (tskIDLE_PRIORITY + 7)
#define RADIO_TASK_PRIORITY          (tskIDLE_PRIORITY + 5)
#define TELEMETRY_TASK_PRIORITY      (tskIDLE_PRIORITY + 3)

// Periods, in milliseconds, chosen from docs/architecture.md's Milestone-1 ranges. This requires
// CONFIG_FREERTOS_HZ raised to 1000 (see ../sdkconfig.defaults) - at the ESP-IDF stock 100Hz/10ms
// tick, a 2ms period would round down to 0 ticks via pdMS_TO_TICKS(), which is not a real 500Hz
// cadence, just a busy-loop.
#define SAFETY_TASK_PERIOD_MS         10  // 100 Hz - see docs/architecture.md
#define SENSOR_TASK_PERIOD_MS          2  // 500 Hz - low end of the documented 500Hz-1kHz range;
                                           // see sensor_task.c for why 500Hz was picked over 1kHz
#define FLIGHT_CONTROL_TASK_PERIOD_MS  4  // 250 Hz - low end of the documented 250-500Hz range
#define RADIO_TASK_PERIOD_MS          20  // 50 Hz - mid of the documented 20-100Hz range
#define TELEMETRY_TASK_PERIOD_MS      50  // 20 Hz - mid of the documented 10-50Hz range

// EstimatorTask has no independent period - per docs/architecture.md it blocks on the sensor
// sample queue and runs immediately when SensorTask publishes a new sample, to avoid adding
// latency between "sample exists" and "estimate reflects it". This value only bounds how long a
// single xQueueReceive() call waits before looping back around (to feed the watchdog and re-block)
// if SensorTask stalls; it is not a target rate.
#define ESTIMATOR_TASK_QUEUE_WAIT_MS  10

// SensorTask reads the IMU every cycle but the barometer only every Nth cycle, since baro dynamics
// are far slower than attitude dynamics (see docs/hardware.md#bmp581-milestone-4). 10 cycles @
// 2ms/cycle = 20ms = 50Hz baro read rate, matching the ~50Hz figure docs/architecture.md's task
// table already names for barometer reads.
#define SENSOR_TASK_BARO_DECIMATION   10

#ifdef __cplusplus
}
#endif

#endif // BICOPTER_MAIN_TASK_CONFIG_H
