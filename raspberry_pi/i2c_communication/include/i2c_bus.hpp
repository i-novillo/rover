#include 
#include <string>
#include <unordered_map>

class I2C_Bus {

private:
    std::unordered_map<std::string, I2C_Device> devices;

public:
    I2C_Bus();
    void register_device(std::string device_name, int device_addr);
    void remove_device(std::string device_name);
    void write_data(std::string device_name, char* data, unsigned data_size);
    void read_data(std::string device_name, char* buffer, unsigned buffer_size);

};