#include "i2c_manager/i2c_device.hpp"
#include "rclcpp/rclcpp.hpp"
#include <pigpiod_if2.h>
#include <iostream>

int I2C_Device::pigpio_initialized = 0;
int I2C_Device::pi = -1;

I2C_Device::I2C_Device(unsigned bus, unsigned device_address) : address(device_address), handle(-1) {

    if (pigpio_initialized == 0) {
        pi = pigpio_start("0.0.0.0", "8888"); // Start pigpio daemon
        if (pi < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("i2c_manager"), "pigpio initialization failed: %d", pi);
            return;
        }
        pigpio_initialized = 1;
    }

    if (pi >= 0) {
        if ((handle = i2c_open(pi, bus, device_address, 0)) < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("i2c_manager"), "I2C Device with address: %u could not be opened.", device_address);
        } else {
            RCLCPP_INFO(rclcpp::get_logger("i2c_manager"), "I2C Device with address: %u opened successfully.", device_address);
        }
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("i2c_manager"), "pigpio not initialized, cannot open I2C device.");
    }
}
/* 
I2C_Device::~I2C_Device() {
    close_device();
    if (pigpio_initialized == 1) { // Only stop pigpio once, at the end.
        pigpio_stop(pi);
        pigpio_initialized = 0;
    }
} */

void I2C_Device::write_data(char* data, unsigned data_size) {

    int result = i2c_write_device(pi, handle, data, data_size);
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
    
    if (i2c_read_device(pi, handle, buffer, buffer_size) < 1) {
        // TODO: Replace for log
        RCLCPP_ERROR(rclcpp::get_logger("i2c_manager"), "Failed to read from I2C device in address: %u", address);
    }
}

void I2C_Device::close_device() {
    if (handle >= 0) {
        if (i2c_close(pi, handle)) {
            RCLCPP_ERROR(rclcpp::get_logger("i2c_manager"), "Failed to close I2C device in address: %u", address);
        }
        handle = -1;
    }
}