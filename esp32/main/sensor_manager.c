#include "sensor_manager.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/i2c_master.h"

#define I2C_MASTER_SCL_IO           13         // TODO: Make configurable
#define I2C_MASTER_SDA_IO           14         // TODO: Make configurable
#define I2C_MASTER_NUM              I2C_NUM_0  // TODO: Make configurable
#define I2C_MASTER_FREQ_HZ          400000     // TODO: Make configurable
#define I2C_MASTER_TX_BUF_DISABLE   0          // TODO: Make configurable
#define I2C_MASTER_RX_BUF_DISABLE   0          // TODO: Make configurable
#define I2C_MASTER_TIMEOUT_MS       1000

#define AS5600_SENSOR_ADDR         0x36 // TODO: Make configruable
#define TCA_ADDR                   0x70 // TODO: Make configruable

#define I2C_TIMEOUT 10                  // ms TODO: make configurable
#define RETRY_SENSOR_SETUP_TIMEOUT 100  // ms TODO: make configurable

#define SENSOR_SAMPLE_FREQ 200 // TODO: make configurable

#define TAG "SENSOR_MANAGER"

static const uint8_t encoder_channels[] = {0, 1, 2, 7}; // TODO: Make configurable if possible (maybe a constant for each array position)

static encoder_data_t encoder_buffer_a;
static encoder_data_t encoder_buffer_b;

static encoder_data_t* encoder_write_buf;
static encoder_data_t* encoder_read_buf;

static i2c_master_bus_handle_t i2c_bus_handle;
static i2c_master_dev_handle_t encoder_handle;
static i2c_master_dev_handle_t tca_handle;

static int32_t motor_counts[MOTOR_COUNT] = {0, 0, 0, 0};
static uint16_t last_raw[MOTOR_COUNT] = {0, 0, 0, 0};
static int64_t last_meas_time = 0;

static const uint8_t as5600_raw_angle_register = 0x0C;

static const float rad_per_count =
    (2.0f * PI) / ((float)AS5600_RESOLUTION * (float)GEAR_RATIO);

static bool tca_select(uint8_t tca_channel)
{
    if (tca_channel > 7) return false;

    uint8_t data = (1 << tca_channel);

    esp_err_t err = i2c_master_transmit(
        tca_handle,
        &data,
        1,
        I2C_TIMEOUT
    );

    return err == ESP_OK;
}

static bool as5600_read_angle(uint16_t* angle, uint8_t encoder_channel)
{
    if (!tca_select(encoder_channel)) {
        ESP_LOGE(TAG, "Could not select channel %d in the I2C multiplexor", encoder_channel);
        return false;
    }

    uint8_t buf[2];

    esp_err_t err = i2c_master_transmit_receive(encoder_handle, &as5600_raw_angle_register, 1, buf, 2, I2C_TIMEOUT);

    if (err == ESP_OK) {
        *angle = ((buf[0] << 8) | buf[1]) & 0x0FFF;
        return true;
    }else {
        ESP_LOGE(TAG, "Error when reading encoder in channel %d", encoder_channel);
        return false;
    }
}

static bool read_encoder_data(void)
{
    uint16_t raw_angle[MOTOR_COUNT];
    bool result = false;

    for (int i = 0; i < MOTOR_COUNT; i++) {
        result = as5600_read_angle(&raw_angle[i], encoder_channels[i]);

        if (!result) {
            ESP_LOGE(TAG, "Could not read encoder %d", i);
            break;
        }
    }

    int64_t now = esp_timer_get_time();
    int64_t dt = now - last_meas_time;
    if (dt <= 0) dt = 1;

    if (result) {
        for (int i = 0; i < MOTOR_COUNT; i++) {
            int32_t diff = (int32_t)raw_angle[i] - (int32_t)last_raw[i];

            if (diff > AS5600_HALF_ROTATION)
                diff -= AS5600_RESOLUTION;
            else if (diff < -AS5600_HALF_ROTATION)
                diff += AS5600_RESOLUTION;

            last_raw[i] = raw_angle[i];

            motor_counts[i] += diff;

            float motor_position = motor_counts[i] * rad_per_count;

            encoder_write_buf->motor_positions[i] = motor_position;
            
            float motor_velocity =
                diff * rad_per_count * 1e6f / (float)dt;
            
            encoder_write_buf->motor_velocities[i] = motor_velocity;
        }
    }

    encoder_write_buf->valid_data = result;
    encoder_write_buf->timestamp = now;
    last_meas_time = now;

    encoder_data_t* tmp_buf = encoder_read_buf;
    encoder_read_buf = encoder_write_buf;
    encoder_write_buf = tmp_buf;

    return result;
}

static void xSensorSamplingTask(void *pvParameters)
{
    TickType_t last_wake_time = xTaskGetTickCount();
 
    while (1) {
        read_encoder_data();
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(MS / SENSOR_SAMPLE_FREQ));
    }
}

bool setup_sensor_manager()
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus_handle));

    i2c_device_config_t encoder_dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AS5600_SENSOR_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    i2c_device_config_t tca_dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = TCA_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus_handle, &encoder_dev_config, &encoder_handle));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus_handle, &tca_dev_config, &tca_handle));

    encoder_write_buf = &encoder_buffer_a;
    encoder_read_buf  = &encoder_buffer_b;
    last_meas_time = esp_timer_get_time();
    
    bool init_ok = false;

    for (int i = 0; i < 5; i++) {
        if (read_encoder_data()) {
            init_ok = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(RETRY_SENSOR_SETUP_TIMEOUT));
    }

    return init_ok;
}

void start_sensor_manager()
{
    xTaskCreate(xSensorSamplingTask, "Sensor Manager", 4096, NULL, 3, NULL);
}

bool get_encoder_data(encoder_data_t* out)
{
    if (out == NULL) return false;
    *out = *encoder_read_buf;
    return true;
}