"""
robot.launch.py
~~~~~~~~~~~~~~~
Full hardware bringup for the personal competition robot.

Launch order:
  1. robot_description  — URDF -> robot_state_publisher
  2. robot_drivers      — TB6612FNG motor driver + odometry
  3. robot_effectors    — DSServo paddle/crank controller
  4. robot_navigation   — Nav2 + LiDAR (optional)

Usage:
  ros2 launch robot_bringup robot.launch.py
  ros2 launch robot_bringup robot.launch.py use_nav:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    use_nav_arg = DeclareLaunchArgument(
        "use_nav", default_value="true",
        description="Launch Nav2 navigation stack",
    )
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz", default_value="true",
        description="Launch RViz2",
    )

    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("robot_description"), "launch", "description.launch.py"]
            )
        )
    )

    drivers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("robot_drivers"), "launch", "drivers.launch.py"]
            )
        )
    )

    effectors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("robot_effectors"), "launch", "effectors.launch.py"]
            )
        )
    )

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("robot_navigation"), "launch", "navigation.launch.py"]
            )
        ),
        condition=IfCondition(LaunchConfiguration("use_nav")),
        launch_arguments={
            "use_rviz":      LaunchConfiguration("use_rviz"),
            "use_sim_time":  "false",
        }.items(),
    )

    return LaunchDescription([
        use_nav_arg,
        use_rviz_arg,
        description_launch,
        drivers_launch,
        effectors_launch,
        nav_launch,
    ])
