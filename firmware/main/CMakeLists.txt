# Milestone 6: the FreeRTOS task architecture. main.c is the composition root (queue/mutex/timer
# creation, task creation); each task_*.c is one task's body; safety_state.c/.h is the one
# genuinely cross-task mutex-protected state (see safety_state.h); task_util.c/.h is the one piece
# of pure logic pulled out for host testing (see ../../tests/task_util_test.c).
idf_component_register(SRCS "main.c"
                             "sensor_task.c"
                             "estimator_task.c"
                             "flight_control_task.c"
                             "radio_task.c"
                             "telemetry_task.c"
                             "safety_task.c"
                             "safety_state.c"
                             "task_util.c"
                        INCLUDE_DIRS "."
                        REQUIRES sensors driver esp_timer)
