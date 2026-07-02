#include "telecommand_interface.h"

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/i2c_slave.h"
#include "driver/i2c_types.h"
#include "constants.h"

#define I2C_SLAVE_SCL_IO            15         // TODO: Make configurable
#define I2C_SLAVE_SDA_IO            16         // TODO: Make configurable
#define ESP_SLAVE_ADDR              0x01       // TODO: Make configurable
#define I2C_SLAVE_NUM               I2C_NUM_1  // TODO: Make configurable
#define I2C_TIMEOUT                 40         // ms TODO: Make configurable
#define MAX_I2C_TIMEOUT_COUNT       25         // TODO: Make configurable

#define TAG "TELECOMMAND_INTERFACE"

typedef struct __attribute__((packed)) {
    uint8_t msg_id;
    uint8_t body[I2C_TC_BODY_SIZE];
} telecommand_t;


static motor_velocities_t motor_velocities_buffer_a;
static motor_velocities_t motor_velocities_buffer_b;

static motor_velocities_t* motor_velocities_write_buf;

static i2c_slave_dev_handle_t slave_handle;
static TaskHandle_t tc_interface_handle;
static telecommand_t latest_tc;

static uint8_t i2c_timeout_count = 0;

static TaskHandle_t motor_controller_handle;

static bool i2c_slave_receive_cb(i2c_slave_dev_handle_t i2c_slave, const i2c_slave_rx_done_event_data_t *evt_data, void *arg)
{
    if (evt_data->length < sizeof(telecommand_t)) return false;

    telecommand_t tmp;
    memcpy(&tmp, evt_data->buffer, sizeof(tmp));

    latest_tc = tmp;

    BaseType_t xTaskWoken = 0;
    vTaskNotifyGiveFromISR(tc_interface_handle, &xTaskWoken);
    
    return xTaskWoken;
}

static void process_latest_tc(void) {
    telecommand_t tmp = latest_tc;

    switch (tmp.msg_id) {
        case 1:
            motor_velocities_t motor_velocities;

            memcpy(&motor_velocities, tmp.body, sizeof(motor_velocities));
            
            *motor_velocities_write_buf = motor_velocities;

            motor_velocities_t* tmp_buf = motor_velocities_read_buf;
            motor_velocities_read_buf = motor_velocities_write_buf;
            motor_velocities_write_buf = tmp_buf;
            
            BaseType_t xTaskWoken = pdFALSE;
            vTaskNotifyGive(motor_controller_handle, &xTaskWoken);
            vportYIELD(xTaskWoken);

            break;
        default:
            break;
    }
}

static void xTelecommandInterfaceTask(void *pvParameters)
{
    while (1) {
        if (ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(I2C_TIMEOUT)) == pdTRUE) {
            ESP_LOGD(TAG, "I2C message received");
            process_latest_tc();
            i2c_timeout_count = 0;
        } else {
            i2c_timeout_count++;
            if (i2c_timeout_count > MAX_I2C_TIMEOUT_COUNT) {
                motor_velocities_t zero = {0};

                *motor_velocities_write_buf = zero;

                motor_velocities_t* tmp = motor_velocities_read_buf;
                motor_velocities_read_buf = motor_velocities_write_buf;
                motor_velocities_write_buf = tmp;
            }
        }
    }
}

bool setup_telecommand_interface(void) {

    i2c_slave_config_t i2c_slv_config = {
    .i2c_port = I2C_SLAVE_NUM,
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .scl_io_num = I2C_SLAVE_SCL_IO,
    .sda_io_num = I2C_SLAVE_SDA_IO,
    .slave_addr = ESP_SLAVE_ADDR,
    .send_buf_depth = 100,
    .receive_buf_depth = 100,
    };

    i2c_slave_event_callbacks_t cbs = {
        .on_receive = i2c_slave_receive_cb,
    };

    ESP_ERROR_CHECK(i2c_new_slave_device(&i2c_slv_config, &slave_handle));
    ESP_ERROR_CHECK(i2c_slave_register_event_callbacks(slave_handle, &cbs, NULL));

    motor_velocities_write_buf = &motor_velocities_buffer_a;
    motor_velocities_read_buf  = &motor_velocities_buffer_b;

    return true;
}

void start_telecommand_interface()
{
    xTaskCreate(xTelecommandInterfaceTask, "Telecommand Interface", 4096, NULL, 3, &tc_interface_handle);
}
