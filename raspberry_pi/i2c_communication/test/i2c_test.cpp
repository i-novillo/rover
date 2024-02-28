#include <pigpio.h>
#include <iostream>
#include "i2c_bus.hpp"
#include "logger.h"

int main() {
    // Start the logger
    Logger::initialise_logger("test", "test", ".");
    Logger::info("Test");

    // Initialize the pigpio library.
    if (gpioInitialise() < 0) {
        Logger::error("pigpio initialization failed");
        return 1;
    }

    I2C_Bus bus;
    std::string arduino_device = "Arduino";
    bus.register_device(arduino_device, 0x01);
    
    char message[] = {0x05, 0x06};
    while(true) {
        bus.write_data(arduino_device, message, 2);
    }

    // Terminate the pigpio library.
    gpioTerminate();

    return 0;
}