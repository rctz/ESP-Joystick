#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory("esp_joystick_ros2")
    
    # Default path to config file
    config_file = os.path.join(
            pkg_dir,
            'config',
            'esp_joystick_params.yaml'
        )
        
    params_file = LaunchConfiguration('params_file')

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
            'params_file', 
            default_value=config_file, 
            description='Full path to the ROS2 parameters file to use'
        ))
    
    # Create node
    esp_joystick_node = Node(
        package="esp_joystick_ros2",
        executable="esp_joystick_node",
        name="esp_joystick_node",
        output="screen",
        parameters=[params_file], 
        emulate_tty=True,
    )
    
    ld.add_action(esp_joystick_node)

    return ld