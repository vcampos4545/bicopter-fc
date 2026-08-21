#include "radio_state.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static SemaphoreHandle_t s_mutex;
static radio_state_t s_state;

esp_err_t radio_state_init(void)
{
    memset(&s_state, 0, sizeof(s_state));

    s_mutex = xSemaphoreCreateMutex();
    return (s_mutex != NULL) ? ESP_OK : ESP_ERR_NO_MEM;
}

void radio_state_set(const radio_state_t *state)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_state = *state;
    xSemaphoreGive(s_mutex);
}

void radio_state_get(radio_state_t *out_state)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    *out_state = s_state;
    xSemaphoreGive(s_mutex);
}
