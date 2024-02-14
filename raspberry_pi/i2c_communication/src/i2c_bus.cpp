#include <i2c_bus.hpp>

I2C_Bus::I2C_Bus() : devices() {}

void I2C_Bus::register_device(std::string device_name, int device_addr) {
    I2CDevice device(device_addr);
    devices.insert({device_name, device});
}

void I2C_Bus::remove_device(std::string device_name) {
    devices.erase(device_name)
}

void I2C_Bus::write_data(std::string device_name, char* data, unsigned data_size) {

}

void I2C_Bus::read_data(std::string device_name, char* buffer, unsigned buffer_size) {

}