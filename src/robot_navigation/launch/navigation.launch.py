"""
navigation.launch.py
~~~~~~~~~~~~~~~~~~~~
Brings up Nav2 with competition-tuned params and an optional pre-built map.
Also starts the RPLidar node if a lidar_device parameter is provided.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_pkg  = get_package_share_directory("robot_navigation")
    nav2_pkg = get_package_share_directory("nav2_bringup")

    params_file = os.path.join(nav_pkg, "config", "nav2_params.yaml")

    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="false")
    use_rviz_arg     = DeclareLaunchArgument("use_rviz",     default_value="true")
    map_arg = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(nav_pkg, "maps", "competition_map.yaml"),
        description="Full path to map yaml file",
    )
    # Set to e.g. /dev/ttyUSB0 to auto-start the RPLidar driver
    lidar_device_arg = DeclareLaunchArgument(
        "lidar_device",
        default_value="",
        description="Serial device for RPLidar (empty = don't launch lidar node)",
    )

    # ── RPLidar node (optional) ───────────────────────────────────────────
    # Requires: sudo apt install ros-humble-rplidar-ros
    rplidar_node = Node(
        package="rplidar_ros",
        executable="rplidar_composition",
        output="screen",
        parameters=[{
            "serial_port":     LaunchConfiguration("lidar_device"),
            "serial_baudrate": 115200,
            "frame_id":        "lidar_link",
            "inverted":        False,
            "angle_compensate": True,
        }],
        condition=IfCondition(
            # Only launch if lidar_device is non-empty
            # We abuse IfCondition with a non-empty string as truthy
            LaunchConfiguration("lidar_device"),
        ),
    )

    # ── Nav2 ─────────────────────────────────────────────────────────────
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "map":          LaunchConfiguration("map"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "params_file":  params_file,
        }.items(),
    )

    # ── RViz ─────────────────────────────────────────────────────────────
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg, "launch", "rviz_launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        launch_arguments={"use_sim_time": LaunchConfiguration("use_sim_time")}.items(),
    )

    return LaunchDescription([
        use_sim_time_arg,
        use_rviz_arg,
        map_arg,
        lidar_device_arg,
        rplidar_node,
        nav2_launch,
        rviz_launch,
    ])
