#include <memory>
#include <pigpio.h>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/i2_c.hpp"
#include "i2c_manager/i2c_bus.hpp"

class I2CManager : public rclcpp::Node
{
    public:
        I2CManager()
        : Node("i2c_manager"), bus()
        {
            gpioInitialise();
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
            for(int i=0; i<4; i++) {
                int16_t speed = i2c_msg.motor_speeds[i];
                char* speed_bytes = reinterpret_cast<char*>(&speed);
                bus.write_data("Arduino", speed_bytes, sizeof(int16_t));
            }
        }

        rclcpp::Subscription<interfaces::msg::I2C>::SharedPtr i2c_messages_;
        I2C_Bus bus;
        
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<I2CManager>());
    gpioTerminate();
    rclcpp::shutdown();
    return 0;
}