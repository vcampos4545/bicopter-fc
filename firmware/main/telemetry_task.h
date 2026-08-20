#ifndef BICOPTER_MAIN_TELEMETRY_TASK_H
#define BICOPTER_MAIN_TELEMETRY_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

// Woken by an esp_timer periodic callback (see main.c) via xTaskNotifyGive() rather than pacing
// itself with vTaskDelayUntil() - see main.c for why this is the one task in this milestone driven
// by esp_timer instead of a plain periodic delay.
void telemetry_task(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif // BICOPTER_MAIN_TELEMETRY_TASK_H
