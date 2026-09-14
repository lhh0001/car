#!/usr/bin/env python3
"""Test 01: real mapping with RViz and a low-speed keyboard controller."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('vehicle_bringup')
    mapping_share = get_package_share_directory('vehicle_mapping')

    mapping_launch = os.path.join(
        bringup_share, 'launch', 'real_mapping.launch.py')
    rviz_config = os.path.join(
        mapping_share, 'config', 'real_mapping.rviz')

    return LaunchDescription([
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
        DeclareLaunchArgument(
            'lidar_port_name',
            default_value='/dev/ttyACM0',
            description='USB LiDAR serial device'),
        DeclareLaunchArgument(
            'host',
            default_value='192.168.4.1',
            description='ESP32 Wi-Fi address'),
        DeclareLaunchArgument(
            'max_linear',
            default_value='0.05',
            description='Maximum keyboard linear speed in m/s'),
        DeclareLaunchArgument(
            'max_angular',
            default_value='1.0',
            description='Maximum keyboard angular speed in rad/s'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mapping_launch),
            launch_arguments={
                'host': LaunchConfiguration('host'),
                'lidar_transport': 'serial',
                'lidar_port_name': LaunchConfiguration('lidar_port_name'),
            }.items(),
        ),

        TimerAction(period=4.0, actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz',
                output='screen',
                arguments=['-d', rviz_config],
            ),
        ]),

        TimerAction(period=5.0, actions=[
            ExecuteProcess(
                cmd=[
                    'gnome-terminal',
                    '--wait',
                    '--title=Vehicle Keyboard Teleop (I/J/K/L)',
                    '--',
                    'bash',
                    '-c',
                    [
                        'exec ros2 run teleop_twist_keyboard '
                        'teleop_twist_keyboard --ros-args ',
                        '-p speed:=', LaunchConfiguration('max_linear'),
                        ' -p turn:=', LaunchConfiguration('max_angular'),
                    ],
                ],
                output='screen',
            ),
        ]),
    ])
