#ifndef I2C_MANAGER_H
#define I2C_MANAGER_H

#include <Wire.h>
#include "motor_controller.h"
#include <AS5600.h>

class I2C_Manager {
    public:
        I2C_Manager(const int device_address, Motor_Controller& motor_controller) 
            : i2c_address(device_address), 
              motor_controller(motor_controller), 
              last_msg_time(0) {}

        void parse_i2c_msg(int howMany) {
            char raw_bytes[32]; 
            int byte_count = 0;

            while (Wire.available() && byte_count < 32) {
                raw_bytes[byte_count++] = Wire.read();
            }

            Serial.println("Move message received");

            if (byte_count != 32) return;

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

            // 👇 Read encoder every time we check timeout
            read_encoder();
        }

        void i2c_setup() {
            Wire.begin(i2c_address);
            Wire.onReceive(I2C_Manager::static_onReceive);
            instance = this;

            delay(100); // let I2C settle

            // --- Encoder init ---
            tcaSelect(ENCODER_CHANNEL);
            delay(10);

            last_raw = encoder.readAngle();

            output_zero_offset =
                (last_raw * 360.0 / COUNTS_PER_REV) / GEAR_RATIO;

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
        static const uint8_t ENCODER_CHANNEL = 2;

        long motor_revs = 0;
        uint16_t last_raw = 0;
        float output_zero_offset = 0.0;

        void tcaSelect(uint8_t i) {
            if (i > 7) return;

            Wire.beginTransmission(TCA_ADDR);
            Wire.write(1 << i);
            Wire.endTransmission();
        }

        void read_encoder() {
            tcaSelect(ENCODER_CHANNEL);

            uint16_t raw = encoder.readAngle();

            int diff = raw - last_raw;

            if (diff > 2048) {
                motor_revs--;
            }
            else if (diff < -2048) {
                motor_revs++;
            }

            last_raw = raw;

            float motor_angle_deg =
                (motor_revs * 360.0) +
                (raw * 360.0 / COUNTS_PER_REV);

            float output_angle_deg =
                (motor_angle_deg / GEAR_RATIO) - output_zero_offset;

            float plot_angle = fmod(output_angle_deg, 360.0);
            if (plot_angle < 0) plot_angle += 360.0;

            // Debug output (optional)
            Serial.print(plot_angle);
            Serial.print(" ");
            Serial.print(0);
            Serial.print(" ");
            Serial.println(360);
        }

        static void static_onReceive(int howMany) {
            if (instance) {
                instance->parse_i2c_msg(howMany);
            }
        }
};

I2C_Manager* I2C_Manager::instance = nullptr;

#endif