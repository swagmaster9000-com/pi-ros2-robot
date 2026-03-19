"""
sim.launch.py
~~~~~~~~~~~~~
Simulation bringup — mirrors robot.launch.py but uses Gazebo
instead of hardware drivers and sets use_sim_time=true throughout.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    use_nav_arg  = DeclareLaunchArgument("use_nav",  default_value="true")
    use_rviz_arg = DeclareLaunchArgument("use_rviz", default_value="true")
    world_arg    = DeclareLaunchArgument(
        "world", default_value="competition_arena",
        description="World name from robot_simulation/worlds/",
    )

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("robot_simulation"), "launch", "gazebo.launch.py"]
            )
        ),
        launch_arguments={"world": LaunchConfiguration("world")}.items(),
    )

    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("robot_description"), "launch", "description.launch.py"]
            )
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("robot_navigation"), "launch", "navigation.launch.py"]
            )
        ),
        condition=IfCondition(LaunchConfiguration("use_nav")),
        launch_arguments={
            "use_sim_time": "true",
            "use_rviz":     LaunchConfiguration("use_rviz"),
        }.items(),
    )

    return LaunchDescription([
        use_nav_arg, use_rviz_arg, world_arg,
        sim_launch,
        description_launch,
        nav_launch,
    ])
