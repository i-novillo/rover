# Use an ARM64v8 Ubuntu base image
FROM arm64v8/ubuntu:22.04

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Install necessary packages
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    crossbuild-essential-arm64 \
    qemu-user-static \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Set up ROS 2 environment
WORKDIR /rover/raspberry_pi/ros2_central_ws

# ROS2 Humble installation https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
# Set locale
RUN apt update && apt install locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && export LANG=en_US.UTF-8

# Setup sources
RUN apt install -y software-properties-common \
    && add-apt-repository universe

RUN apt update && apt install curl -y \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 packages
RUN apt update -y && apt upgrade -y

RUN apt install -y \
    ros-humble-ros-base \
    ros-dev-tools

# Environment setup
#RUN . /opt/ros/humble/setup.bash && echo ". /opt/ros/humble/setup.bash" >> ~/.bashrc \
RUN export ROS_DOMAIN_ID=0 && echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc \
    && export ROS_LOCALHOST_ONLY=1 && echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc

# Install pigpio
COPY external_dependencies/pigpio/master.zip /rover/pigpio.zip
RUN apt install -y unzip \
    && unzip /rover/pigpio.zip -d /rover \
    && rm /rover/pigpio.zip \
    && cd /rover/pigpio-master \
    && make \
    && make install \
    && rm -rf /rover/pigpio-master

# Copy the entrypoint script into the container
COPY cross_compile_entrypoint.sh /usr/local/bin/cross_compile_entrypoint.sh

# Ensure it has execution permissions
RUN chmod +x /usr/local/bin/cross_compile_entrypoint.sh

# Set the entrypoint
ENTRYPOINT ["/usr/local/bin/cross_compile_entrypoint.sh"]

