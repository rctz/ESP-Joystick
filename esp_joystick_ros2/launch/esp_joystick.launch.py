#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("esp_joystick_ros2")

    # Declare launch arguments
    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/ttyUSB0",
        description="Serial port for ESP32 communication",
    )

    baud_rate_arg = DeclareLaunchArgument(
        "baud_rate",
        default_value="115200",
        description="Baud rate for serial communication",
    )

    publish_rate_arg = DeclareLaunchArgument(
        "publish_rate",
        default_value="10.0",
        description="Joy message publishing rate in Hz",
    )

    max_buttons_arg = DeclareLaunchArgument(
        "max_button_count",
        default_value="16",
        description="Maximum number of buttons in joy message",
    )

    max_axes_arg = DeclareLaunchArgument(
        "max_axis_count",
        default_value="8",
        description="Maximum number of axes in joy message",
    )

    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Log level (debug, info, warn, error)",
    )

    # Create node
    esp_joystick_node = Node(
        package="esp_joystick_ros2",
        executable="esp_joystick_node",
        name="esp_joystick_node",
        output="screen",
        parameters=[
            {
                "serial_port": LaunchConfiguration("serial_port"),
                "baud_rate": LaunchConfiguration("baud_rate"),
                "publish_rate": LaunchConfiguration("publish_rate"),
                "max_button_count": LaunchConfiguration("max_button_count"),
                "max_axis_count": LaunchConfiguration("max_axis_count"),
            }
        ],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        emulate_tty=True,
    )

    return LaunchDescription(
        [
            serial_port_arg,
            baud_rate_arg,
            publish_rate_arg,
            max_buttons_arg,
            max_axes_arg,
            log_level_arg,
            esp_joystick_node,
        ]
    )
