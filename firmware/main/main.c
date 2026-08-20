// main/: FreeRTOS entry point. Milestone 2 scope only — boot and log, nothing armed yet.
// Later milestones slot sensor/actuator/radio init between the startup log and the idle loop
// below, per the boot sequence recorded in docs/firmware.md.

#include "esp_idf_version.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "bicopter-fc";

void app_main(void)
{
    // BOOT
    ESP_LOGI(TAG, "bicopter-fc firmware starting (ESP-IDF %s)", esp_get_idf_version());

    // -> (nothing else armed yet): sensors, actuators, and radio are later milestones.
    ESP_LOGI(TAG, "boot: no sensors/actuators/radio initialized yet (milestone 2 scope)");

    // -> log ready
    ESP_LOGI(TAG, "bicopter-fc ready");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
