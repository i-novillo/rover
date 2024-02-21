#include "../include/i2c_bus.hpp"
#include "../include/i2c_device.hpp"
//#include "../../logger/logger.h"

I2C_Bus::I2C_Bus() : devices() {}

void I2C_Bus::register_device(std::string device_name, int device_addr) {
    this->devices.emplace(std::make_pair(device_name, I2C_Device(1, device_addr)));
}

void I2C_Bus::remove_device(std::string device_name) {
    this->devices[device_name].close_device();
    this->devices.erase(device_name);
}

void I2C_Bus::write_data(std::string device_name, char* data, unsigned data_size) {
    this->devices[device_name].write_data(data, data_size);
}

void I2C_Bus::read_data(std::string device_name, char* buffer, unsigned buffer_size) {
    this->devices[device_name].read_data(buffer, buffer_size);
}