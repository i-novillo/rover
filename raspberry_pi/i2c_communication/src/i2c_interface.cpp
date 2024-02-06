#include <pigpio.h>
#include <iostream>

int main() {
    // Initialize the pigpio library.
    if (gpioInitialise() < 0) {
        std::cerr << "pigpio initialization failed" << std::endl;
        return 1;
    }

    int handle;

    handle = i2cOpen(1, 0x01, 0);

    if (handle < 0) {
        std::cerr << "Failed to open I2C device" << std::endl;
        gpioTerminate();
        return 1;
    }

    char data[] = {0x01, 0x02};
    while(true) {
        int result = i2cWriteDevice(handle, data, 2);
        if (result < 0) {
            std::cerr << "Failed to write to I2C device" << std::endl;
        } else {
            std::cout << "Motor speed command sent successfully" << std::endl;
        }
        gpioSleep(PI_TIME_RELATIVE, 1, 0);
    }

    // Close the I2C device handle.
    i2cClose(handle);

    // Terminate the pigpio library.
    gpioTerminate();

    return 0;
}