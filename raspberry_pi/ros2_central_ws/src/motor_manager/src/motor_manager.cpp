#include <unordered_map>
#include <vector>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/motor_input.hpp"
#include "interfaces/msg/i2_c.hpp"

// Speeds are multiplied by a factor of 100 to avoid floating point numbers
std::unordered_map<std::string, std::array<int16_t,4>> direction_to_motor_speeds  = {
    {"forward", {100, 100, 100, 100}},
    {"backward", {-100, -100, -100, -100}},
    {"left", {-100, 100, -100, 100}},
    {"right", {100, -100, 100, -100}},
    {"stop", {0, 0, 0, 0}}
};

class MotorManager : public rclcpp::Node
{
    public:
        MotorManager()
        : Node("motor_manager")
        {
            motor_input_ = this->create_subscription<interfaces::msg::MotorInput>(
                "motor_input", 10, std::bind(&MotorManager::received_motor_input, this, std::placeholders::_1));
                RCLCPP_INFO(this->get_logger(), "Motor Manager Node started");

            i2c_messages_ = this->create_publisher<interfaces::msg::I2C>("i2c_messages", 10);
        }

    private:
        void received_motor_input(const interfaces::msg::MotorInput & motor_input_msg) const
        {
            RCLCPP_DEBUG(this->get_logger(), "I heard: '%s'", motor_input_msg.rover_direction.c_str());
            std::array<int16_t,4> motor_speeds;

            if (direction_to_motor_speeds.find(motor_input_msg.rover_direction) != direction_to_motor_speeds.end()) {
                motor_speeds = direction_to_motor_speeds[motor_input_msg.rover_direction];
            } else {
                RCLCPP_WARN(this->get_logger(), "Invalid rover direction. Setting to stop");
                motor_speeds = direction_to_motor_speeds["stop"];
            }

            auto i2c_msg = interfaces::msg::I2C();
            i2c_msg.motor_speeds = motor_speeds;

            this->i2c_messages_->publish(i2c_msg);
        }

        rclcpp::Subscription<interfaces::msg::MotorInput>::SharedPtr motor_input_;
        rclcpp::Publisher<interfaces::msg::I2C>::SharedPtr i2c_messages_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorManager>());
    rclcpp::shutdown();
    return 0;
}