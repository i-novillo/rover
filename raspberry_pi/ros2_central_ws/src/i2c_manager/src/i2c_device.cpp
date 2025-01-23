#include "i2c_manager/i2c_device.hpp"
#include "rclcpp/rclcpp.hpp"
#include <pigpio.h>
#include <iostream>

I2C_Device::I2C_Device(unsigned bus, unsigned device_address) {
    address = device_address;
    if ((handle = i2cOpen(bus, device_address, 0)) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("i2c_manager"), "I2C Device with address: %u could not be opened.", device_address);
    } else {
        RCLCPP_INFO(rclcpp::get_logger("i2c_manager"), "I2C Device with address: %u opened successfully.", device_address);
    }
}

void I2C_Device::write_data(char* data, unsigned data_size) {

    int result = i2cWriteDevice(this->handle, data, data_size);
    if (result < 0) {
        RCLCPP_ERROR(
            rclcpp::get_logger("i2c_manager"),
            "Failed to write to I2C device at address: %u. Error code: %d",
            address,
            result
        );
    } else {
        RCLCPP_DEBUG(rclcpp::get_logger("i2c_manager"), "Data sent successfully to address: %u", address);
    }
    
}

void I2C_Device::read_data(char* buffer, unsigned buffer_size) {
    if(buffer_size < 32) {
        return;
        // TODO: Add error or exception: I can only write to a buffer of the maximum size to prevent data loss
    }
    
    if (i2cReadDevice(handle, buffer, buffer_size) < 1) {
        // TODO: Replace for log
        RCLCPP_ERROR(rclcpp::get_logger("i2c_manager"), "Failed to read from I2C device in address: %u", address);
    }
}

void I2C_Device::close_device(){
    if (i2cClose(handle)) {
        RCLCPP_ERROR(rclcpp::get_logger("i2c_manager"), "Failed to close I2C device in address: %u", address);
    }
}