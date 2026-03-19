"""description.launch.py — publish URDF and start robot_state_publisher."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg = get_package_share_directory("robot_description")
    xacro_file = os.path.join(pkg, "urdf", "robot.urdf.xacro")

    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="false")
    robot_desc = xacro.process_file(xacro_file).toxml()

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_desc},
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    jsp = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    return LaunchDescription([use_sim_time_arg, rsp, jsp])
