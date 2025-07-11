import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='demo5_gui',
            executable='ros_camera',
            name='ros_camera_node'
        ),
        Node(
            package='demo5_gui',
            executable='ros2_interface',
            name='ros2_interface_node'
        ),
    ])