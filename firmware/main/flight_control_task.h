#ifndef BICOPTER_MAIN_FLIGHT_CONTROL_TASK_H
#define BICOPTER_MAIN_FLIGHT_CONTROL_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

// FlightControlTask reads its own pending safety-ping notification via ulTaskNotifyTake() from
// inside its own task context - see safety_task.h for the handle SafetyTask uses to send it, set
// by main() after both tasks are created.
void flight_control_task(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif // BICOPTER_MAIN_FLIGHT_CONTROL_TASK_H
