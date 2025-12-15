# Automatic deployment of ROS2 environment using systemd service

In order for the ROS2 environment to be automatically launched on startup, a service can be created using `systemd`.
For that, copy the service file `ros2_launch.service` into `/etc/systemd/system/` (requires root access).
Then, enable the service with `sudo systemctl enable ros2_launch.service`.