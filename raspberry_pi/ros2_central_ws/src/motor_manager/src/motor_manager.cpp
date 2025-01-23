#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/motor_input.hpp"

class MotorManager : public rclcpp::Node
{
    public:
        MotorManager()
        : Node("motor_manager")
        {
            motor_input_ = this->create_subscription<interfaces::msg::MotorInput>(
                "motor_input", 10, std::bind(&MotorManager::received_motor_input, this, std::placeholders::_1));
                RCLCPP_INFO(this->get_logger(), "Motor Manager Node started");
        }

    private:
        void received_motor_input(const interfaces::msg::MotorInput & motor_input_msg) const
        {
            RCLCPP_INFO(this->get_logger(), "I heard: '%s'", motor_input_msg.rover_direction.c_str());
        }

        rclcpp::Subscription<interfaces::msg::MotorInput>::SharedPtr motor_input_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorManager>());
    rclcpp::shutdown();
    return 0;
}