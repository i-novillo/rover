#!/bin/bash

RASPBERRY_USER="ignacio"
RASPBERRY_IP="192.168.1.40"
ROS2_LOCAL_WORKSPACE="/home/ign/Code/rover/raspberry_pi/ros2_central_ws"
INSTALL_TARGET_FOLDER="/home/ignacio/Code/rover/raspberry_pi/ros2_central_ws"

docker run -v /home/ign/Code/rover:/rover --name ros2_cross_compile --network host --platform linux/arm64 --rm -it ros2_aarch64:latest

ssh "$RASPBERRY_USER@$RASPBERRY_IP" "rm -rf $INSTALL_TARGET_FOLDER/install/*" || { 
    echo "Error: Failed to remove old install directory on Raspberry Pi"; exit 1; 
}

cd "$ROS2_LOCAL_WORKSPACE"

scp -r install "$RASPBERRY_USER@$RASPBERRY_IP:$INSTALL_TARGET_FOLDER"
SCP_RESULT=$?

if [ $EXIT_CODE -ne 0 ]; then
    echo "Error: Failed to copy install directory to Raspberry Pi"
    exit $EXIT_CODE
fi

echo "Deployment completed successfully!"