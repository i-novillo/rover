#include <memory>
#include <pigpiod_if2.h>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/i2_c.hpp"
#include "i2c_manager/i2c_bus.hpp"

const int I2C_MSG_SIZE = 32;
const int I2C_MSG_ID_POSITION = 0;

const int MOTOR_SPEEDS_ID = 0x01;

class I2CManager : public rclcpp::Node
{
    public:
        I2CManager()
        : Node("i2c_manager"), bus()
        {
            i2c_messages_ = this->create_subscription<interfaces::msg::I2C>(
                "i2c_messages", 10, std::bind(&I2CManager::received_i2c_message, this, std::placeholders::_1));
                RCLCPP_INFO(this->get_logger(), "I2C Manager Node started");
            std::string arduino_device = "Arduino";
            bus.register_device(arduino_device, 0x01);
        }

    private:
        void received_i2c_message(const interfaces::msg::I2C & i2c_msg)
        {
            RCLCPP_INFO(this->get_logger(), "I2C message received. Sending to Arduino.");
            char msg[I2C_MSG_SIZE];
            create_i2c_motor_msg(msg, i2c_msg.motor_speeds);
            bus.write_data("Arduino", msg, I2C_MSG_SIZE);
            
        }

        void create_i2c_motor_msg(char * msg, const std::array<int16_t, 4>& motor_speeds) {
            msg[I2C_MSG_ID_POSITION] = MOTOR_SPEEDS_ID;
            for(int i=0; i<4; i++) {
                int16_t speed = motor_speeds[i];
                int motor_position = (2*i) + 1;
                msg[motor_position] = (speed >> 8) & 0xFF;
                msg[motor_position + 1] = speed & 0xFF;
            }
        }

        rclcpp::Subscription<interfaces::msg::I2C>::SharedPtr i2c_messages_;
        I2C_Bus bus;
        
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<I2CManager>());
    rclcpp::shutdown();
    return 0;
}