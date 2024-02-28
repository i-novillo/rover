cmake_minimum_required(VERSION 3.10)
project(rover VERSION 0.1)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# Globally include logger
include_directories(${CMAKE_SOURCE_DIR}/logger)

# I2C Communication component
add_subdirectory(logger)
add_subdirectory(i2c_communication)

# You can add other subdirectories for different components as needed
# add_subdirectory(another_component)