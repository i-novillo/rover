#ifndef I2C_DEVICE_HPP
#define I2C_DEVICE_HPP

class I2C_Device {

private:    
    unsigned address;
    int handle;
    static int pi;
    static int pigpio_initialized;
    
public:
    I2C_Device() = default;
    //~I2C_Device();

    /**
    * @brief Initializes a new device connected via I2C
    *
    * Device objects are used by I2C_Bus to manage I2C communication with multiple
    * devices connected using I2C. Each I2C device stores each address and corresponding
    * handle for sending and receiving data via the Raspberry I2C's ports.
    *
    * @param bus Bus to which the device is connected to
    * @param device_address I2C Address associated to the initialized device
    */
    I2C_Device(unsigned bus, unsigned device_address);

    /**
    * @brief Writes data as bytes to the device's address
    *
    * @param data Pointer to the data buffer being written
    * @param data_size Size (numebr of bytes) of the data buffer
    */
    void write_data(char* data, unsigned data_size);

    /**
    * @brief Reads data as bytes into the specified buffer
    *
    * @param buffer Pointer to the data buffer where read bytes are written
    * @param buffer_size Size (numebr of bytes) of the data buffer
    */
    void read_data(char* buffer, unsigned buffer_size);

    /**
    * @brief Closes I2C communication for the current device
    *
    * To be used when a given device is removed from the current I2C Bus object
    */
    void close_device();
};

#endif