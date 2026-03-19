"""
gazebo.launch.py
~~~~~~~~~~~~~~~~
Launches Gazebo with the competition arena and spawns the personal robot.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim_pkg    = get_package_share_directory("robot_simulation")
    gazebo_pkg = get_package_share_directory("gazebo_ros")

    world_arg = DeclareLaunchArgument(
        "world", default_value="competition_arena",
        description="World name (no extension) from robot_simulation/worlds/",
    )
    x_arg   = DeclareLaunchArgument("x",   default_value="0.0")
    y_arg   = DeclareLaunchArgument("y",   default_value="0.0")
    z_arg   = DeclareLaunchArgument("z",   default_value="0.05")
    yaw_arg = DeclareLaunchArgument("yaw", default_value="0.0")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "world": PathJoinSubstitution(
                [sim_pkg, "worlds", [LaunchConfiguration("world"), ".world"]]
            ),
            "verbose": "false",
        }.items(),
    )

    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=[
                    "-topic", "/robot_description",
                    "-entity", "competition_robot",
                    "-x",  LaunchConfiguration("x"),
                    "-y",  LaunchConfiguration("y"),
                    "-z",  LaunchConfiguration("z"),
                    "-Y",  LaunchConfiguration("yaw"),
                ],
                output="screen",
            )
        ],
    )

    return LaunchDescription([
        world_arg, x_arg, y_arg, z_arg, yaw_arg,
        gazebo,
        spawn_robot,
    ])
