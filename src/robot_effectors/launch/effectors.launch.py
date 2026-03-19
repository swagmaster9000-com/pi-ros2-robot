"""
effectors.launch.py
~~~~~~~~~~~~~~~~~~~
Starts both DSServo effector nodes with shared config.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("robot_effectors"),
        "config", "effector_config.yaml",
    )

    paddle_node = Node(
        package="robot_effectors",
        executable="paddle_controller_node.py",
        name="paddle_controller_node",
        output="screen",
        parameters=[config],
    )

    crank_node = Node(
        package="robot_effectors",
        executable="crank_controller_node.py",
        name="crank_controller_node",
        output="screen",
        parameters=[config],
    )

    return LaunchDescription([paddle_node, crank_node])
