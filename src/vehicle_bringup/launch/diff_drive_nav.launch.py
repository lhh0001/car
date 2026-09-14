#!/usr/bin/env python3
"""Compose simulation, online SLAM and Nav2 autonomous navigation."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch(package, filename):
    return PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory(package), 'launch', filename))


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='world.world',
            description='World filename in vehicle_simulation/worlds'),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        DeclareLaunchArgument('z_pose', default_value='0.05'),
        DeclareLaunchArgument('wheel_radius', default_value='0.08'),
        DeclareLaunchArgument('wheel_separation', default_value='0.30'),
        DeclareLaunchArgument('wheel_width', default_value='0.04'),
        DeclareLaunchArgument('body_length', default_value='0.40'),
        DeclareLaunchArgument('body_width', default_value='0.25'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('keyboard', default_value='true'),
        DeclareLaunchArgument('max_linear', default_value='0.20'),
        DeclareLaunchArgument('max_angular', default_value='1.0'),

        IncludeLaunchDescription(
            _launch('vehicle_simulation', 'simulation.launch.py'),
            launch_arguments={
                'world': LaunchConfiguration('world'),
                'x_pose': LaunchConfiguration('x_pose'),
                'y_pose': LaunchConfiguration('y_pose'),
                'z_pose': LaunchConfiguration('z_pose'),
                'wheel_radius': LaunchConfiguration('wheel_radius'),
                'wheel_separation': LaunchConfiguration('wheel_separation'),
                'wheel_width': LaunchConfiguration('wheel_width'),
                'body_length': LaunchConfiguration('body_length'),
                'body_width': LaunchConfiguration('body_width'),
            }.items(),
        ),
        IncludeLaunchDescription(
            _launch('vehicle_mapping', 'slam.launch.py'),
            launch_arguments={'profile': 'sim', 'startup_delay': '4.0'}.items(),
        ),
        IncludeLaunchDescription(
            _launch('vehicle_navigation', 'navigation.launch.py'),
            launch_arguments={'profile': 'sim', 'startup_delay': '6.0'}.items(),
        ),
        TimerAction(period=7.0, actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz',
                output='screen',
                arguments=[
                    '-d',
                    os.path.join(
                        get_package_share_directory('vehicle_mapping'),
                        'config',
                        'real_mapping.rviz',
                    ),
                ],
                condition=IfCondition(LaunchConfiguration('rviz')),
            ),
        ]),
        TimerAction(period=8.0, actions=[
            ExecuteProcess(
                cmd=[
                    'gnome-terminal',
                    '--wait',
                    '--title=Mapping Keyboard (I/J/K/L)',
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
                condition=IfCondition(LaunchConfiguration('keyboard')),
            ),
        ]),
    ])
