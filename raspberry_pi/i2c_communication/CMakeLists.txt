# I2C Headers
include_directories(PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# spdlog
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../../spdlog/include)

# Test executable
add_executable(i2c_test test/i2c_test.cpp src/i2c_bus.cpp src/i2c_device.cpp)

# If I wanted to add i2c_communication for compilation in a different script as a library
# Add a library target for i2c_communication
#add_library(i2c_communication src/i2c_bus.cpp src/i2c_device.cpp)

# Link pigpio library
target_link_libraries(i2c_test pigpio)

# Link spdlog compilation
find_library(SPDLOG_LIB spdlog HINTS ${CMAKE_CURRENT_SOURCE_DIR}/../../../spdlog)
if(SPDLOG_LIB)
    target_link_libraries(i2c_test ${SPDLOG_LIB})
endif()