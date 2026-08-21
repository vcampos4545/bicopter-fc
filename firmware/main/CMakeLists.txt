# Milestone 6: the FreeRTOS task architecture. main.c is the composition root (queue/mutex/timer
# creation, task creation); each task_*.c is one task's body; safety_state.c/.h is the one
# genuinely cross-task mutex-protected state (see safety_state.h); task_util.c/.h is the one piece
# of pure logic pulled out for host testing (see ../../tests/task_util_test.c). As of Milestone 14,
# radio_task.c wires in the real ESP-NOW `radio` component (see ../components/radio and
# ../../docs/radio.md) behind CONFIG_BICOPTER_RADIO_ENABLED. As of Milestone 15, radio_task.c also
# wires in the CRSF RC-receiver backend behind CONFIG_BICOPTER_CRSF_RADIO_ENABLED, mutually
# exclusive with the ESP-NOW option (enforced by a build-time #error in radio_task.c). As of
# Milestone 16, safety_task.c calls the real flight-mode/arming/failsafe logic from the new
# `safety` component (see ../components/safety and ../../docs/safety.md), safety_state.c/.h now
# carries a real flight_mode_t (not just a placeholder armed bool), and radio_state.c/.h publishes
# RadioTask's latest command/health for SafetyTask to consume - the same mutex-protected-struct
# shape as safety_state.c/.h, for the same reason (see radio_state.h). As of Milestone 18 (see
# ../../docs/bench_test.md), main.c calls actuators_init_safe() behind the new
# CONFIG_BICOPTER_ACTUATORS_ENABLED gate and publishes the result via actuators_state.c/.h (no
# mutex needed - see that header), and bench_test_task.c wires the new `bench_test` component's
# UART console into real firmware state (sensor sample queue, radio_state.h, actuators_state.h) as
# a seventh, non-watchdog-registered task.
idf_component_register(SRCS "main.c"
                             "sensor_task.c"
                             "estimator_task.c"
                             "flight_control_task.c"
                             "radio_task.c"
                             "radio_state.c"
                             "telemetry_task.c"
                             "safety_task.c"
                             "safety_state.c"
                             "actuators_state.c"
                             "bench_test_task.c"
                             "task_util.c"
                        INCLUDE_DIRS "."
                        REQUIRES sensors actuators radio safety power bench_test driver esp_timer)
