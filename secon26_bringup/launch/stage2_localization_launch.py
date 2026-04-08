"""
stage2_localization_launch.py
Stage 2 — Localization stack.
Run after Stage 1 is fully up and sensors are verified.

Verify with:
  ros2 topic hz /odom                  # should be ~50Hz
  ros2 topic hz /odometry/filtered     # should be ~50Hz
  ros2 topic hz /map                   # should publish after driving
  ros2 run tf2_tools view_frames       # check TF tree is complete

First run: drive around arena to build map, then save:
  ros2 run nav2_map_server map_saver_cli -f ~/secon26_maps/arena_map
Then set mode: localization in slam_toolbox_params.yaml
"""

import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():

    pkg_dir  = os.path.dirname(os.path.realpath(__file__))
    pkg_root = os.path.dirname(pkg_dir)

    slam_config = os.path.join(pkg_root, 'config', 'slam_toolbox_params.yaml')
    ekf_config  = os.path.join(pkg_root, 'config', 'ekf_params.yaml')

    # TB6612 motor driver + dead-reckoning odom
    motors = Node(
        package='secon26_bringup',
        executable='tb6612_driver',
        name='tb6612_driver',
        output='screen',
        parameters=[{
            'max_speed':   0.20,
            'track_width': 0.30,
        }]
    )

    # EKF — fuse /odom + /imu/data -> /odometry/filtered
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config,
            {'use_sim_time': False},
        ],
        remappings=[('odometry/filtered', '/odometry/filtered')]
    )

    # SLAM Toolbox
    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config,
            {'use_sim_time': False},
        ],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/odometry/filtered'),
        ]
    )

    return LaunchDescription([
        motors,
        # EKF needs odom + IMU to be publishing
        TimerAction(period=2.0, actions=[ekf]),
        # SLAM needs EKF output + scan
        TimerAction(period=5.0, actions=[slam]),
    ])
