# toolchain-rpi.cmake

# Specify the cross-compiler locations
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

set(CMAKE_C_COMPILER /usr/bin/aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER /usr/bin/aarch64-linux-gnu-g++)

# Adjust these flags for 64-bit compilation as needed
# You might not need specific architecture flags for basic compilation,
# but you can specify optimization flags here.
# Example: set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O2" CACHE STRING "" FORCE)
# Example: set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2" CACHE STRING "" FORCE)

# Specify the root directory for the target (optional)
# If you have a sysroot directory for your target environment, set it here.
# set(CMAKE_FIND_ROOT_PATH /path/to/your/sysroot)

# Direct CMake to use the sysroot for finding libraries, includes, etc.
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
set(CMAKE_INCLUDE_PATH ${CMAKE_INCLUDE_PATH} /usr/local/include)
