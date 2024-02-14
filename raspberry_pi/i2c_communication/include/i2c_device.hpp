class I2C_Device {

private:    
    int address;
    int handle;
    
public:
    I2C_Device(unsigned bus, unsigned device_address) {}
    ~I2C_Device() {}

    void write_data(char* data, unsigned data_size);
    void read_data(char* buffer, unsigned buffer_size);
};