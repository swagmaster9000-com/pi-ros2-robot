"""
stage4_mission_launch.py
Stage 4 — Mission controller and effectors.
Run ONLY after Stages 1, 2, and 3 are all running and Nav2 is active.

This is the final stage that enables autonomous competition operation.
The mission controller sequences all tasks and triggers servo actions.

Verify with:
  ros2 service list | grep servo    # should show duck_collect, crank_turn
  ros2 topic list | grep servo      # should show /servo/paddle, /servo/crank
"""

import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():

    # DSServo driver — paddles and crank
    servos = Node(
        package='secon26_bringup',
        executable='dsservo_driver',
        name='dsservo_driver',
        output='screen',
        parameters=[{
            'paddle_trigger_distance': 0.5,
            'sweep_duration':          0.5,
            'crank_duration':          1.0,
        }]
    )

    # Mission controller — sequences all competition tasks
    mission = Node(
        package='secon26_bringup',
        executable='secon26_mission_controller',
        name='mission_controller',
        output='screen',
    )

    return LaunchDescription([
        servos,
        # Mission controller starts after servos are ready
        TimerAction(period=3.0, actions=[mission]),
    ])
