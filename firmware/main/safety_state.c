#include "safety_state.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static SemaphoreHandle_t s_mutex;
static safety_state_t s_state;

esp_err_t safety_state_init(void)
{
    s_state.armed = false;
    s_state.checked_at_us = 0;

    s_mutex = xSemaphoreCreateMutex();
    return (s_mutex != NULL) ? ESP_OK : ESP_ERR_NO_MEM;
}

void safety_state_set(bool armed, int64_t checked_at_us)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_state.armed = armed;
    s_state.checked_at_us = checked_at_us;
    xSemaphoreGive(s_mutex);
}

void safety_state_get(safety_state_t *out_state)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    *out_state = s_state;
    xSemaphoreGive(s_mutex);
}
