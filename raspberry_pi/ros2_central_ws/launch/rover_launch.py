from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='i2c_manager',
            namespace='i2c_manager',
            executable='i2c_manager',
            name='i2c_manager'
        ),
        Node(
            package='motor_manager',
            namespace='motor_manager',
            executable='motor_manager',
            name='motor_manager'
        )
    ])