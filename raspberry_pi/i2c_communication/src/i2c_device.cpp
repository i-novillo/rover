#include "i2c_device.hpp"
//#include "../../logger/logger.h"
#include <pigpio.h>
#include <iostream>

I2C_Device::I2C_Device(unsigned bus, unsigned device_address) {
    address = device_address;
    if ((handle = i2cOpen(bus, device_address, 0) < 0)){
        // TODO: Change for error/warning log
        throw std::runtime_error("I2C Device with address: " + std::to_string(device_address) + " could not be opened.");
    } 
}

void I2C_Device::write_data(char* data, unsigned data_size) {

    if (i2cWriteDevice(this->handle, data, data_size) < 0) {
        // TODO: Change for error/warning log
        std::cerr << "Failed to write to I2C device in address: " << address << std::endl;
    } else {
        // TODO: Change for info/debug log
        std::cout << "Motor speed command sent successfully" << std::endl;
    }
    
}

void I2C_Device::read_data(char* buffer, unsigned buffer_size) {
    if(buffer_size < 32) {
        return;
        // TODO: Add error or exception: I can only write to a buffer of the maximum size to prevent data loss
    }
    
    if (i2cReadDevice(handle, buffer, buffer_size) < 1) {
        // TODO: Replace for log
        std::cerr << "Failed to read from I2C device in address: " << address << std::endl;
    }
}

void I2C_Device::close_device(){
    if (i2cClose(handle)) {
        // TODO: Replace for log
        std::cerr << "Failed to close I2C device in address: " << address << std::endl;
    }
}