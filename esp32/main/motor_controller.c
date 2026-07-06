#include "motor_controller.h"

#include <string.h>
#include <math.h>

#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define MOTOR_CONTROLLER_FREQ       1000.0f      // Hz                          // TODO: Make configurable
#define MOTOR_CONTROLLER_PERIOD_S   (1.0f / MOTOR_CONTROLLER_FREQ)              // TODO: Make configurable

#define PWM_MODE                    LEDC_HIGH_SPEED_MODE                        // TODO: Make configurable
#define PWM_FREQ                    15000                                       // TODO: Make configurable
#define PWM_MAX_DUTY                1023         // 10-bit                      // TODO: Make configurable

#define MAX_ACCEL                   20.0f        // rad/s²                      // TODO: Make configurable

#define BATTERY_VOLTAGE             12.0f        // TODO: Measure with ADC      // TODO: Make configurable

// --------------------- These values should be handled per motor, and configurable
#define KS 0.25f            // TODO: Make configurable
#define KV 0.12f            // TODO: Make configurable
#define KA 0.02f            // TODO: Make configurable

#define KP 0.25f            // TODO: Make configurable
#define KD 0.12f            // TODO: Make configurable
#define KI 0.02f            // TODO: Make configurable

#define PID_INTEGRAL_LIMIT 20 // TODO: Make configurable

#define MOTOR_1_PWM_PIN_FORWARD  1   // TODO: Make configurable
#define MOTOR_2_PWM_PIN_FORWARD  2   // TODO: Make configurable
#define MOTOR_3_PWM_PIN_FORWARD  3   // TODO: Make configurable
#define MOTOR_4_PWM_PIN_FORWARD  4   // TODO: Make configurable
#define MOTOR_1_PWM_PIN_BACKWARD 5   // TODO: Make configurable
#define MOTOR_2_PWM_PIN_BACKWARD 6   // TODO: Make configurable
#define MOTOR_3_PWM_PIN_BACKWARD 7   // TODO: Make configurable
#define MOTOR_4_PWM_PIN_BACKWARD 8   // TODO: Make configurable
// -------------------------------------------------------------------------------

#define MAX_INVALID_ENCODER_MEASUREMENTS 500

#define TAG "MOTOR_CONTROLLER"

static const ledc_channel_t motor_pwm_channels_forward[MOTOR_COUNT] =
{
    LEDC_CHANNEL_0,
    LEDC_CHANNEL_1,
    LEDC_CHANNEL_2,
    LEDC_CHANNEL_3
};

static const gpio_num_t motor_pwm_pins_forward[MOTOR_COUNT] = 
{
    MOTOR_1_PWM_PIN_FORWARD,
    MOTOR_2_PWM_PIN_FORWARD,
    MOTOR_3_PWM_PIN_FORWARD,
    MOTOR_4_PWM_PIN_FORWARD
};

static const ledc_channel_t motor_pwm_channels_backward[MOTOR_COUNT] =
{
    LEDC_CHANNEL_4,
    LEDC_CHANNEL_5,
    LEDC_CHANNEL_6,
    LEDC_CHANNEL_7
};

static const gpio_num_t motor_pwm_pins_backward[MOTOR_COUNT] = 
{
    MOTOR_1_PWM_PIN_BACKWARD,
    MOTOR_2_PWM_PIN_BACKWARD,
    MOTOR_3_PWM_PIN_BACKWARD,
    MOTOR_4_PWM_PIN_BACKWARD
};

typedef struct
{
    float commanded_velocity;
    float reference_acceleration;
    float desired_velocity;
    float measured_velocity;
    
} motion_reference_t;

static motion_reference_t motor_refs[MOTOR_COUNT] = {0};

typedef struct
{
    float error;
    float previous_error;
    float integral_error;
    float derivative_error;

} pid_state_t;

static pid_state_t pid[MOTOR_COUNT] = {0};

static TaskHandle_t motor_controller_handle;

bool setup_motor_controller() {
    // Configure PWM timer
    ledc_timer_config_t timer = {
        .speed_mode      = PWM_MODE,
        .timer_num       = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz         = PWM_FREQ,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    // Configure PWM channels, and ensure they start at 0
    for (int i = 0; i < MOTOR_COUNT; ++i)
    {
        // Motor forward channels
        ledc_channel_config_t forward_channel = {
            .gpio_num   = motor_pwm_pins_forward[i],
            .speed_mode = PWM_MODE,
            .channel    = motor_pwm_channels_forward[i],
            .timer_sel  = LEDC_TIMER_0,
            .duty       = 0,
            .hpoint     = 0
        };

        ESP_ERROR_CHECK(ledc_channel_config(&forward_channel));
        ledc_set_duty(PWM_MODE, motor_pwm_channels_forward[i], 0);
        ledc_update_duty(PWM_MODE,  motor_pwm_channels_forward[i]);

        // Motor backward channels
        ledc_channel_config_t backward_channel = {
            .gpio_num   = motor_pwm_pins_backward[i],
            .speed_mode = PWM_MODE,
            .channel    = motor_pwm_channels_backward[i],
            .timer_sel  = LEDC_TIMER_0,
            .duty       = 0,
            .hpoint     = 0
        };

        ESP_ERROR_CHECK(ledc_channel_config(&backward_channel));
        ledc_set_duty(PWM_MODE, motor_pwm_channels_backward[i], 0);
        ledc_update_duty(PWM_MODE,  motor_pwm_channels_backward[i]);
    }

    return true;
}

static void update_trajectory(int motor_idx)
{
    float velocity_error = motor_refs[motor_idx].desired_velocity - motor_refs[motor_idx].commanded_velocity;

    if (fabsf(velocity_error) > 1e-6f)
    {
        float error = motor_refs[motor_idx].desired_velocity - motor_refs[motor_idx].commanded_velocity;
        float max_step = MAX_ACCEL * MOTOR_CONTROLLER_PERIOD_S;

        if (fabsf(error) <= max_step)
        {
            motor_refs[motor_idx].reference_acceleration = error / MOTOR_CONTROLLER_PERIOD_S;
            motor_refs[motor_idx].commanded_velocity = motor_refs[motor_idx].desired_velocity;
        }
        else
        {
            motor_refs[motor_idx].reference_acceleration = copysignf(MAX_ACCEL, error);
            motor_refs[motor_idx].commanded_velocity += motor_refs[motor_idx].reference_acceleration * MOTOR_CONTROLLER_PERIOD_S;
        }
    }
    else
    {
        motor_refs[motor_idx].reference_acceleration = 0.0f;
    }    
}


static float compute_feedforward_input(int motor_idx)
{
    float v = motor_refs[motor_idx].commanded_velocity;
    float a = motor_refs[motor_idx].reference_acceleration;

    return KS * sgn(v) + KV * v + KA * a;
}

static float compute_feedback_input(int motor_idx)
{
    // P-Term
    pid[motor_idx].error = motor_refs[motor_idx].commanded_velocity - motor_refs[motor_idx].measured_velocity;
    
    // D-Term
    pid[motor_idx].derivative_error = (pid[motor_idx].error - pid[motor_idx].previous_error) / MOTOR_CONTROLLER_PERIOD_S;
    pid[motor_idx].previous_error = pid[motor_idx].error;
    
    // I-Term
    pid[motor_idx].integral_error += pid[motor_idx].error * MOTOR_CONTROLLER_PERIOD_S;
    if (pid[motor_idx].integral_error > PID_INTEGRAL_LIMIT)
    {
        pid[motor_idx].integral_error = PID_INTEGRAL_LIMIT;
    }
    else if (pid[motor_idx].integral_error < -PID_INTEGRAL_LIMIT)
    {
        pid[motor_idx].integral_error = -PID_INTEGRAL_LIMIT;
    }

    return KP * pid[motor_idx].error + KD * pid[motor_idx].derivative_error + KI * pid[motor_idx].integral_error;

}

static void apply_motor_voltage(int motor_idx, float voltage)
{
    if (voltage > BATTERY_VOLTAGE)
    {
        voltage = BATTERY_VOLTAGE;
    }
    else if (voltage < -BATTERY_VOLTAGE)
    {
        voltage = -BATTERY_VOLTAGE;
    }

    float duty = fabsf(voltage) / BATTERY_VOLTAGE;

    if (duty > 1.0f)
    {
        duty = 1.0f;
    }

    uint32_t pwm_input = (uint32_t)(duty * PWM_MAX_DUTY);

    if (sgn(voltage) > 0.0f)
    {
        ledc_set_duty(PWM_MODE, motor_pwm_channels_forward[motor_idx], pwm_input);
        ledc_set_duty(PWM_MODE, motor_pwm_channels_backward[motor_idx], 0);
    }
    else if (voltage < 0.0f)
    {
        ledc_set_duty(PWM_MODE, motor_pwm_channels_forward[motor_idx], 0);
        ledc_set_duty(PWM_MODE, motor_pwm_channels_backward[motor_idx], pwm_input);
    }
    else
    {
        ledc_set_duty(PWM_MODE, motor_pwm_channels_forward[motor_idx], 0);
        ledc_set_duty(PWM_MODE, motor_pwm_channels_backward[motor_idx], 0);
    }
}

static void xMotorControllerTask(void *pvParameters)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    int invalid_encoder_count = 0;
 
    while (1) {

        // Check if new motor telecommand has been received
        if (ulTaskNotifyTake(pdTRUE, 0) == pdTRUE) {
            motor_velocities_t desired_motor_velocities;;
            memcpy(&desired_motor_velocities, motor_velocities_read_buf, sizeof(desired_motor_velocities));
            for (int i = 0; i < MOTOR_COUNT; i++)
            {
                motor_refs[i].desired_velocity = (float)desired_motor_velocities.value[i];
            }
        }
        
        // Update latest velocities measurement
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
                    ledc_set_duty(PWM_MODE, motor_pwm_channels_forward[i], 0);
                    ledc_set_duty(PWM_MODE, motor_pwm_channels_backward[i], 0);
                    ledc_update_duty(PWM_MODE, motor_pwm_channels_forward[i]);
                    ledc_update_duty(PWM_MODE, motor_pwm_channels_backward[i]);
                }
                continue;
            }
        }
        else
        {
            invalid_encoder_count = 0;
            for (int i = 0; i < MOTOR_COUNT; i++)
            {
                motor_refs[i].measured_velocity = (float)latest_encoder_reading.motor_velocities[i];
            }
        }
        
        // Compute PWM inputs
        for (int i = 0; i < MOTOR_COUNT; i++)
        {
            update_trajectory(i);

            float ff_control_input = compute_feedforward_input(i);
            float fb_control_input = compute_feedback_input(i);

            float voltage_control_input = ff_control_input + fb_control_input;
            apply_motor_voltage(i, voltage_control_input);
        }

        // Update PWM output
        for (int i = 0; i < MOTOR_COUNT; i++)
        {
            ledc_update_duty(PWM_MODE, motor_pwm_channels_forward[i]);
            ledc_update_duty(PWM_MODE, motor_pwm_channels_backward[i]);
        }

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(MS / MOTOR_CONTROLLER_FREQ));
    }
}

void start_motor_controller()
{
    xTaskCreate(xMotorControllerTask, "Motor Controller", 4096, NULL, 4, &motor_controller_handle);
}

TaskHandle_t get_motor_controller_handle()
{
    return motor_controller_handle;
}