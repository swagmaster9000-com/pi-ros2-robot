"""drivers.launch.py — starts the TB6612FNG motor driver node with config."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("robot_drivers"),
        "config", "tb6612_config.yaml",
    )

    motor_driver = Node(
        package="robot_drivers",
        executable="tb6612_motor_driver_node.py",
        name="tb6612_motor_driver_node",
        output="screen",
        parameters=[config],
    )

    return LaunchDescription([motor_driver])
