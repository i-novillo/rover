#!/bin/bash

set -e

cd /rover/raspberry_pi/ros2_central_ws
rm -rf build install log || { echo "Error: Failed to remove old build directories"; exit 1; }

source /opt/ros/humble/setup.bash
colcon build --merge-install || { echo "Error: colcon build failed"; exit 1; }