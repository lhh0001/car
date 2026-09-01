#!/usr/bin/env python3
"""Optional manual-control and graph-monitor tools for a running robot."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('teleop', default_value='true'),
        DeclareLaunchArgument('monitor', default_value='false'),
        TimerAction(period=3.0, actions=[
            Node(
                package='vehicle_control',
                executable='graph_monitor.py',
                name='graph_monitor',
                output='screen',
                condition=IfCondition(LaunchConfiguration('monitor')),
            ),
        ]),
        TimerAction(period=5.0, actions=[
            Node(
                package='vehicle_control',
                executable='teleop_gui.py',
                name='teleop_gui',
                output='screen',
                condition=IfCondition(LaunchConfiguration('teleop')),
            ),
        ]),
    ])
