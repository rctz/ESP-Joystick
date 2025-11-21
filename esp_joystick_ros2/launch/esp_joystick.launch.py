#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_dir = get_package_share_directory("esp_joystick_ros2")
    
    config_file = os.path.join(
            pkg_dir,
            'config',
            'esp_joystick_params.yaml'
        )
        
    params_file = LaunchConfiguration('params_file')
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
            'params_file', default_value=config_file, description='Full path to the ROS2 parameters file to use'
        ))
    
    param_substitutions = {}
    
    configured_params = RewrittenYaml(
            source_file=params_file,
            param_rewrites=param_substitutions,
            convert_types=True)

    # Create node
    esp_joystick_node = Node(
        package="esp_joystick_ros2",
        executable="esp_joystick_node",
        name="esp_joystick_node",
        output="screen",
        parameters=[configured_params],
        emulate_tty=True,
    )
    
    ld.add_action(esp_joystick_node)

    return ld
