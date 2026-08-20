// SensorTask - Milestone 6 scaffolding. Calls the real Milestone 3/4 driver interfaces
// (mpu6050_read()/bmp581_read()) when CONFIG_BICOPTER_SENSORS_ENABLED is set (a board with
// sensors actually wired up); no board has been chosen yet (see docs/hardware.md), so that option
// defaults to disabled and this task instead runs its full loop/queue/watchdog plumbing while
// skipping I2C entirely, publishing an all-invalid sample each cycle rather than crashing on an
// absent bus. See AGENTS.md "What's real vs. stub right now" for the exact verified-vs-deferred
// line for this milestone.
//
// Target rate: 500Hz (SENSOR_TASK_PERIOD_MS = 2, see task_config.h), the low end of
// docs/architecture.md's documented 500Hz-1kHz range. 500Hz was picked over 1kHz specifically
// because it is achievable with a clean integer tick count at this project's 1000Hz FreeRTOS tick
// (2 ticks/period, see sdkconfig.defaults) with headroom for scheduling jitter; 1kHz would demand
// a 1-tick period with zero slack, which is the kind of number that should be picked after
// measuring real I2C transaction time on hardware (per this milestone's brief), not asserted now.
// This should be revisited once real MPU6050 timing is measured on real hardware (milestone 17).
#include <string.h>

#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sdkconfig.h"

#include "bmp581.h"
#include "bmp581_registers.h"
#include "mpu6050.h"
#include "mpu6050_registers.h"

#include "sensor_task.h"
#include "task_config.h"
#include "task_util.h"

static const char *TAG = "sensor_task";

#if CONFIG_BICOPTER_SENSORS_ENABLED
// Real hardware init path. Reviewed against the driver headers, not exercised: no physical
// MPU6050/BMP581 was available in this environment, and CONFIG_BICOPTER_SENSORS_ENABLED defaults
// to "n" for exactly that reason (see docs/hardware.md's open board/pin items). Whoever wires up a
// real board flips this Kconfig option and fills in the I2C pin/address config below it.
static esp_err_t sensor_hw_init(mpu6050_handle_t *out_imu, bmp581_handle_t *out_baro)
{
    *out_imu = NULL;
    *out_baro = NULL;

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = CONFIG_BICOPTER_I2C_PORT,
        .sda_io_num = CONFIG_BICOPTER_I2C_SDA_GPIO,
        .scl_io_num = CONFIG_BICOPTER_I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t i2c_bus;
    esp_err_t err = i2c_new_master_bus(&bus_cfg, &i2c_bus);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "i2c_new_master_bus failed: %s (no sensor bus this boot)",
                 esp_err_to_name(err));
        return err;
    }

    mpu6050_config_t imu_cfg = {
        .i2c_bus = i2c_bus,
        .i2c_address = MPU6050_I2C_ADDR_AD0_LOW,
        .i2c_clock_hz = CONFIG_BICOPTER_I2C_CLOCK_HZ,
        .data_ready_gpio = CONFIG_BICOPTER_MPU6050_INT_GPIO,
        .read_timeout_ms = SENSOR_TASK_PERIOD_MS,
        .convert = {
            .accel_range = MPU6050_ACCEL_RANGE_8G,
            .gyro_range = MPU6050_GYRO_RANGE_1000DPS,
            .dlpf_bandwidth = MPU6050_DLPF_44HZ,
            .sample_rate_div = 0,
            .accel_offset_mps2 = {.x = 0, .y = 0, .z = 0},
            .gyro_offset_radps = {.x = 0, .y = 0, .z = 0},
        },
    };
    err = mpu6050_init(&imu_cfg, out_imu);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "mpu6050_init failed: %s", esp_err_to_name(err));
        *out_imu = NULL;
    }

    bmp581_config_t baro_cfg = {
        .i2c_bus = i2c_bus,
        .i2c_address = BMP581_I2C_ADDR_SDO_LOW,
        .i2c_clock_hz = CONFIG_BICOPTER_I2C_CLOCK_HZ,
        .odr_reg_value = BMP581_ODR_FASTEST,
        .convert = {
            .sea_level_pa = 101325.0f, // ISA standard; not correct for a real deployment field -
                                        // see docs/hardware.md#bmp581-milestone-4
            .filter_alpha = 0.2f,
        },
    };
    err = bmp581_init(&baro_cfg, out_baro);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "bmp581_init failed: %s", esp_err_to_name(err));
        *out_baro = NULL;
    }

    // An I2C bus with nothing responding on it is the expected "no hardware" outcome in this
    // environment; only treat init as having genuinely failed if neither sensor came up.
    return (*out_imu != NULL || *out_baro != NULL) ? ESP_OK : ESP_FAIL;
}
#endif // CONFIG_BICOPTER_SENSORS_ENABLED

void sensor_task(void *pvParameters)
{
    sensor_task_params_t *params = (sensor_task_params_t *)pvParameters;

    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    mpu6050_handle_t imu = NULL;
    bmp581_handle_t baro = NULL;

#if CONFIG_BICOPTER_SENSORS_ENABLED
    if (sensor_hw_init(&imu, &baro) != ESP_OK) {
        ESP_LOGW(TAG, "no sensor responded on the configured I2C bus; running without hardware");
    } else {
        ESP_LOGI(TAG, "sensor hardware initialized (imu=%s baro=%s)", imu ? "yes" : "no",
                 baro ? "yes" : "no");
    }
#else
    ESP_LOGI(TAG,
             "CONFIG_BICOPTER_SENSORS_ENABLED=n (no board chosen yet, see docs/hardware.md); "
             "running the task/queue/watchdog loop without real sensor I/O");
#endif

    sensor_sample_t sample;
    memset(&sample, 0, sizeof(sample));

    uint32_t cycle = 0;
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        cycle++;

        if (imu != NULL) {
            esp_err_t err = mpu6050_read(imu, &sample.imu);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "mpu6050_read failed: %s", esp_err_to_name(err));
                sample.imu.valid = false;
            }
        } else {
            sample.imu.valid = false;
            sample.imu.timestamp_us = esp_timer_get_time();
        }

        if (task_should_decimate(cycle, SENSOR_TASK_BARO_DECIMATION)) {
            if (baro != NULL) {
                esp_err_t err = bmp581_read(baro, &sample.baro);
                if (err != ESP_OK) {
                    ESP_LOGW(TAG, "bmp581_read failed: %s", esp_err_to_name(err));
                    sample.baro.valid = false;
                }
            } else {
                sample.baro.valid = false;
                sample.baro.timestamp_us = esp_timer_get_time();
            }
        }

        // Latest-value handoff (see sensor_task.h) - never blocks even if EstimatorTask is slow
        // to drain the previous sample.
        xQueueOverwrite(params->sample_queue, &sample);

        if ((cycle % 500) == 1) { // ~once/second at 500Hz - avoid flooding the log at task rate
            ESP_LOGI(TAG, "cycle=%lu imu.valid=%d baro.valid=%d", (unsigned long)cycle,
                     sample.imu.valid, sample.baro.valid);
        }

        esp_task_wdt_reset();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SENSOR_TASK_PERIOD_MS));
    }
}
