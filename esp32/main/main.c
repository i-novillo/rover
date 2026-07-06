#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "sensor_manager.h"
#include "telecommand_interface.h"
#include "motor_controller.h"

#define TAG "MAIN"

void app_main(void)
{
    bool setup_succesful = true;

    if (!setup_sensor_manager()) {
        ESP_LOGE(TAG, "Sensor manager setup failed");
        setup_succesful = false;
    }

    if (!setup_telecommand_interface()) {
        ESP_LOGE(TAG, "Telecommand interface setup failed");
        setup_succesful = false;
    }

    if (!setup_motor_controller()) {
        ESP_LOGE(TAG, "Motor Controller setup failed");
        setup_succesful = false;
    }

    if (!setup_succesful) {
        ESP_LOGE(TAG, "Critical subsystem setup failed. Restarting...");
        vTaskDelay(pdMS_TO_TICKS(500));
        esp_restart();
    }

    start_sensor_manager();
    start_motor_controller();
    TaskHandle_t motor_controller_handle = get_motor_controller_handle();
    start_telecommand_interface(motor_controller_handle);
    
}
