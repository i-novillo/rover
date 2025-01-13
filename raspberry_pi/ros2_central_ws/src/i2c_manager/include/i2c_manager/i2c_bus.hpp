#include <string>
#include <unordered_map>
#include "i2c_device.hpp"

class I2C_Bus {

private:
    std::unordered_map<std::string, I2C_Device> devices;

public:

    /**
    * @brief Initializes a new I2C Bus object
    *
    * An I2C Bus is responsible for handling all the communications with devices
    * connected via I2C to the Raspberry. The I2C Bus allows registering multiple 
    * devices for I2C communication, such that sending and receiving data with these 
    * devices can be performed by assigning a unique name.
    * 
    * By default, I2C_Bus is initialized to bus 0 in the Raspberry.
    */
    I2C_Bus();

    /**
    * @brief Adds a new device to the list of available devices in the current bus
    *
    * @param device_name Name to be assigned for the new device
    * @param device_address Address associated with the new device
    */
    void register_device(std::string device_name, int device_addr);
    
    /**
    * @brief Removes a pre-registered device fromm the list of available devices in the current bus
    *
    * @param device_name String identifier name of the device to be removed
    */
    void remove_device(std::string device_name);

    /**
    * @brief Writes a buffer of bytes to the desired device
    *
    * @param device_name Name of the device I2C_Bus is writing to
    * @param data Pointer to the buffer being sent
    * @param data_size Size (number of bytes) of the buffer being sent
    */
    void write_data(std::string device_name, char* data, unsigned data_size);

    /**
    * @brief Reads data from a desired device into the specified buffer
    *
    * @param device_name Name of the device I2C_Bus is reading from
    * @param buffer Pointer to the buffer that will store the data
    * @param buffer_size Number of bytes being read
    */
    void read_data(std::string device_name, char* buffer, unsigned buffer_size);

};