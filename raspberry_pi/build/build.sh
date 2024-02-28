#!/bin/bash

# Navigate to the script's directory (the build directory)
cd "$(dirname "$0")"

# Clean-up: Remove all files in the current directory, preserving the script itself
find . -mindepth 1 -not -name "$(basename "$0")" -exec rm -rf {} \;

# Configure the build environment
# Assuming the toolchain file is in the same directory as the build folder or specify the correct relative path
cmake -B. -H.. -DCMAKE_TOOLCHAIN_FILE=../toolchain-rpi.cmake

# Build the project
cmake --build . -- -j$(nproc)