cmake_minimum_required(VERSION 3.8)
project(i2c_manager)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Enable compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)

# Add include directories
include_directories(include)

# Declare the executable
add_executable(i2c_manager
  src/i2c_bus.cpp
  src/i2c_device.cpp
  src/i2c_manager.cpp
)

# Link dependencies
ament_target_dependencies(i2c_manager rclcpp interfaces)

# Link pigpio library
target_link_libraries(i2c_manager pigpiod_if2)

# Install header files
install(
  DIRECTORY include/
  DESTINATION include
)

# Install the executable
install(
  TARGETS i2c_manager
  DESTINATION lib/${PROJECT_NAME}
)

# Mark this package as an ament package
ament_package()