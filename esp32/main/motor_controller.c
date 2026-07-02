#include "motor_controller.h"

#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define MOTOR_CONTROLLER_FREQ       1000.0f      // Hz                          // TODO: Make configurable
#define MOTOR_CONTROLLER_PERIOD_S   (1.0f / MOTOR_CONTROLLER_FREQ)              // TODO: Make configurable

#define PWM_MODE                    LEDC_HIGH_SPEED_MODE                        // TODO: Make configurable
#define PWM_FREQ                    15000                                       // TODO: Make configurable
#define PWM_MAX_DUTY                1023         // 10-bit                      // TODO: Make configurable

#define MAX_ACCEL                   20.0f        // rad/s²                      // TODO: Make configurable

#define BATTERY_VOLTAGE             12.0f        // TODO: Measure with ADC      // TODO: Make configurable

#define KS 0.25f            // TODO: Make configurable
#define KV 0.12f            // TODO: Make configurable
#define KA 0.02f            // TODO: Make configurable

#define MOTOR_1_PWM_PIN 1   // TODO: Make configurable
#define MOTOR_2_PWM_PIN 2   // TODO: Make configurable
#define MOTOR_3_PWM_PIN 3   // TODO: Make configurable
#define MOTOR_4_PWM_PIN 4   // TODO: Make configurable

#define MAX_INVALID_ENCODER_MEASUREMENTS 500

#define TAG "MOTOR_CONTROLLER"

typedef enum {
    MOTOR_1 = 0,
    MOTOR_2,
    MOTOR_3,
    MOTOR_4
} motor_id_t;

static const ledc_channel_t motor_pwm_channels[MOTOR_COUNT] =
{
    LEDC_CHANNEL_0,
    LEDC_CHANNEL_1,
    LEDC_CHANNEL_2,
    LEDC_CHANNEL_3
};

static const gpio_num_t motor_pwm_pins[MOTOR_COUNT] = 
{
    MOTOR_1_PWM_PIN,
    MOTOR_2_PWM_PIN,
    MOTOR_3_PWM_PIN,
    MOTOR_4_PWM_PIN
};

typedef struct
{
    float velocity;
    float acceleration;
} motion_reference_t;

// TODO: Match the typing here 
// **********
static motor_velocities_t desired_motor_velocities = {0};
static int32_t measured_motor_velocities[4] = {0};
// **********

static motion_reference_t motor_refs[MOTOR_COUNT] = {0};

void setup_motor_controller() {
    // Configure PWM timer
    ledc_timer_config_t timer = {
        .speed_mode      = PWM_MODE,
        .timer_num       = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz         = 15000,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    // Configure PWM channels, and ensure they start at 0
    for (int i = 0; i < MOTOR_COUNT; ++i)
    {
        ledc_channel_config_t channel = {
            .gpio_num   = motor_pwm_pins[i],
            .speed_mode = PWM_MODE,
            .channel    = motor_pwm_channels[i],
            .timer_sel  = LEDC_TIMER_0,
            .duty       = 0,
            .hpoint     = 0
        };

        ESP_ERROR_CHECK(ledc_channel_config(&channel));
        ledc_set_duty(PWM_MODE, motor_pwm_channels[i], 0);
        ledc_update_duty(PWM_MODE,  motor_pwm_channels[i]);
    }
}

static void update_velocity_profile(int motor_idx)
{
    float error = desired_motor_velocities[motor_idx] - motor_ref[motor_idx].velocity;
    float max_step = MAX_ACCEL * MOTOR_CONTROLLER_PERIOD_S;

    if (fabsf(error) <= max_step)
    {
        motor_ref[motor_idx].acceleration = error / MOTOR_CONTROLLER_PERIOD_S;
        motor_ref[motor_idx].velocity = desired_motor_velocities[motor_idx];
    }
    else
    {
        motor_ref[motor_idx].acceleration = copysignf(MAX_ACCEL, error);
        motor_ref[motor_idx].velocity += motor_ref[motor_idx].acceleration * MOTOR_CONTROLLER_PERIOD_S;
    }
}


static float compute_feedforward_input(int motor_idx)
{
    float v = motor_ref[motor_idx].velocity;
    float a = motor_ref[motor_idx].acceleration;

    return KS * sgn(v) + KV * v + KA * a;
}

static float compute_feedback_input(int motor_idx)
{
// TODO: Implement
}

static void xSensorSamplingTask(void *pvParameters)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    int invalid_encoder_count = 0;
 
    while (1) {

        if (ulTaskNotifyTake(pdTRUE, 0) == pdTRUE) {
            memcpy(&desired_motor_velocities, *motor_velocities_read_buf, sizeof(desired_motor_velocities));
        }
        
        encoder_data_t latest_encoder_reading;
        bool encoder_ok = (get_encoder_data(&latest_encoder_reading) && latest_encoder_reading.valid_data);

        if (!encoder_ok)
        {
            invalid_encoder_count++;

            if (invalid_encoder_count >= MAX_INVALID_ENCODER_MEASUREMENTS)
            {
                ESP_LOGE(TAG, "Encoder data lost. Stopping motor controller.");
                for (int i = 0; i < MOTOR_COUNT; i++)
                {
                    ledc_set_duty(PWM_MODE, motor_pwm_channels[i], 0);
                    ledc_update_duty(PWM_MODE, motor_pwm_channels[i]);
                }
                continue;
            }
        }
        else
        {
            invalid_encoder_count = 0;
            measured_motor_velocities = latest_encoder_reading.motor_velocities;
        }

        for (int i = 0; i < MOTOR_COUNT; i++)
        {
            float error = desired_motor_velocities[i] - motor_ref[i].velocity;

            if (fabsf(error) > 1e-6f)
            {
                update_velocity_profile(i);
            }
            else
            {
                motor_ref[i].acceleration = 0.0f;
            }

            float ff_control_input = compute_feedforward_input(i);
            float fb_control_input = compute_feedback_input(i);

            float voltage_control_input = ff_control_input + fb_control_input;
            if (voltage_control_input > BATTERY_VOLTAGE)
            {
                voltage_control_input = BATTERY_VOLTAGE;
            }
            else if (voltage_control_input < -BATTERY_VOLTAGE)
            {
                voltage_control_input = -BATTERY_VOLTAGE;
            }

            float duty = fabsf(voltage_control_input) / BATTERY_VOLTAGE;

            if (duty > 1.0f)
            {
                duty = 1.0f;
            }

            uint32_t pwm_input = (uint32_t)(duty * PWM_MAX_DUTY);

            ledc_set_duty(PWM_MODE, motor_pwm_channels[i], pwm_input);
        }

        for (int i = 0; i < MOTOR_COUNT; i++)
        {
            ledc_update_duty(PWM_MODE, motor_pwm_channels[i]);
        }

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(MS / MOTOR_CONTROLLER_FREQ));
    }
}