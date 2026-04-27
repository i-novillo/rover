#ifndef I2C_MANAGER_H
#define I2C_MANAGER_H

#include <Wire.h>
#include "motor_controller.h"
#include <AS5600.h>

static const uint8_t encoder_channels[4] = {0, 1, 2, 7};

class I2C_Manager {
    public:
        I2C_Manager(const int device_address, Motor_Controller& motor_controller) 
            : i2c_address(device_address), motor_controller(motor_controller), last_msg_time(0) {};

        void parse_i2c_msg(int howMany) {
            char raw_bytes[32]; 
            int byte_count = 0;

            while (Wire.available() && byte_count < 32) {
                raw_bytes[byte_count++] = Wire.read();
            }

            if (byte_count != 32) {
                return;
            }
        
            // Update timestamp upon receiving a valid message
            last_msg_time = millis();

            int msg_type = (int)raw_bytes[0];
            switch (msg_type) {
                case 1:
                    int motor_speeds[4];
                    for (int i = 0; i < 4; i++) {
                        int motor_index = (i * 2) + 1;
                        int motor_speed = (int)((unsigned char)raw_bytes[motor_index] << 8 | (unsigned char)raw_bytes[motor_index + 1]);
                        motor_speeds[i] = motor_speed;    
                    }
                    this->motor_controller.move_motors(motor_speeds);
                    break;
            }
        };

        void check_timeout() {
            if (millis() - last_msg_time > 1000) {
                int stop_speeds[4] = {0, 0, 0, 0};
                this->motor_controller.move_motors(stop_speeds);
            }

            Serial.print("Encoder readings -> ");
            for (int encoder_index = 0; encoder_index < 4; encoder_index++) {
                read_encoder(encoder_index);
                Serial.print("Motor ");
                Serial.print(encoder_index);
                Serial.print(": ");
                Serial.print(motor_angles[encoder_index]);
                Serial.print("  ");
            }
        }

        void i2c_setup() {
            Wire.begin(i2c_address);
            Wire.onReceive(I2C_Manager::static_onReceive);
            instance = this;

            delay(100);

            // --- Encoder init ---
            for (int encoder_index = 0; encoder_index < 4; encoder_index++) {
                tcaSelect(encoder_index);
                delay(10);

                last_raw[encoder_index] = encoder.readAngle();

                output_zero_offset[encoder_index] =
                    (last_raw[encoder_index] * 360.0 / COUNTS_PER_REV) / GEAR_RATIO;
            }
        
            last_msg_time = millis();
        };

    private:
        const int i2c_address;
        Motor_Controller& motor_controller;
        unsigned long last_msg_time;
        static I2C_Manager* instance;

        // ===== ENCODER =====
        AS5600 encoder;

        static constexpr float GEAR_RATIO = 270.0;
        static constexpr float COUNTS_PER_REV = 4096.0;
        static const uint8_t TCA_ADDR = 0x70;

        long motor_revs[4] = {0, 0, 0, 0};
        uint16_t last_raw[4] = {0, 0, 0, 0};
        float motor_angles[4] = {0.0, 0.0, 0.0, 0.0};
        float output_zero_offset[4] = {0.0, 0.0, 0.0, 0.0};

        void tcaSelect(uint8_t i) {
            if (i > 7) return;

            Wire.beginTransmission(TCA_ADDR);
            Wire.write(1 << i);
            Wire.endTransmission();
        }

        void read_encoder(const uint8_t encoder_index) {
            tcaSelect(encoder_channels[encoder_index]);

            uint16_t raw = encoder.readAngle();

            int diff = raw - last_raw[encoder_index];

            if (diff > 2048) {
                motor_revs[encoder_index]--;
            }
            else if (diff < -2048) {
                motor_revs[encoder_index]++;
            }

            last_raw[encoder_index] = raw;

            float motor_angle_deg =
                (motor_revs[encoder_index] * 360.0) +
                (raw * 360.0 / COUNTS_PER_REV);

            float output_angle_deg =
                (motor_angle_deg / GEAR_RATIO) - output_zero_offset[encoder_index];

            float plot_angle = fmod(output_angle_deg, 360.0);
            if (plot_angle < 0) plot_angle += 360.0;
            motor_angles[encoder_index] = plot_angle;
        }

        static void static_onReceive(int howMany) {
            if (instance) {
                instance->parse_i2c_msg(howMany);
            }
        }
};

I2C_Manager* I2C_Manager::instance = nullptr;

#endif