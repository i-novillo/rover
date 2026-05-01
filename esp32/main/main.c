#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "sensor_manager.h"

#define TAG "MAIN"

void app_main(void)
{
    if (!setup_sensor_manager()) {
        ESP_LOGE(TAG, "Sensor manager failed to initialize. Restarting system in 1s...");

        vTaskDelay(pdMS_TO_TICKS(1000));

        esp_restart();
    }

    start_sensor_manager();
}
