#ifndef I2C_MANAGER_H
#define I2C_MANAGER_H

#include <Wire.h>
#include "motor_controller.h"

class I2C_Manager {
    public:
        I2C_Manager(const int device_address, const Motor_Controller& motor_controller) : i2c_address(device_address), motor_controller(motor_controller) {};

        void parse_i2c_msg(int howMany) {
            char raw_bytes[32]; // Fixed-size array
            int byte_count = 0;

            while (Wire.available() && byte_count < 32) {
                raw_bytes[byte_count++] = Wire.read();
            }

            if (byte_count != 32) {
                Serial.print("Invalid number of motor speeds received: ");
                Serial.println(byte_count);
                return;
            }

            int msg_type = (int)raw_bytes[0];

            switch (msg_type) {
                case 1:
                    Serial.println("Motor speeds received");
                    int motor_speeds[] = {0, 0, 0, 0};
                    for (int i = 0; i < 4; i++) {
                        int motor_index = (i * 2) + 1;
                        int motor_speed = (int)((unsigned char)raw_bytes[motor_index] << 8 | (unsigned char)raw_bytes[motor_index + 1]);
                        motor_speeds[i] = motor_speed;    
                    }
                    this->motor_controller.move_motors(motor_speeds);
                    break;

                default:
                    Serial.println("Unknown I2C message type");
                    break;
            }
        };

        void i2c_setup() {
            Wire.begin(i2c_address);
            Wire.onReceive(I2C_Manager::static_onReceive);
            instance = this;
        };

    private:
        const int i2c_address;
        const Motor_Controller& motor_controller;
        static I2C_Manager* instance;

        static void static_onReceive(int howMany) {
            if (instance) {
                instance->parse_i2c_msg(howMany);
            }
        }
};

I2C_Manager* I2C_Manager::instance = nullptr;

#endif