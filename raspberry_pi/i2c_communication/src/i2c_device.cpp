#include <i2c_device.hpp>
#include <pigpio.h>

I2C_Device::I2C_Device(unsigned bus, unsigned device_address) {
    address = device_address;
    handle = i2cOpen(bus, device_address, 0);
}

I2C_Device::~I2C_Device() {
    i2cClose(handle);
}

void I2C_Device::write_data(char* data, unsigned data_size) {
    int write_result;
    write_result = i2cWriteDevice(handle, data, data_size);
    // TODO: Include logs in the situation in which data transmission fails.
}

void I2C_Device::read_data(char* buffer, unsigned buffer_size) {
    if(buffer_size < 32) {
        return;
        // TODO: Add error or exception: I can only write to a buffer of the maximum size to prevent data loss
    }
    i2cReadDevice(handle, buffer, buffer_size);
}