#!/usr/bin/env python3
"""Compose simulation, online SLAM and Nav2 autonomous navigation."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _launch(package, filename):
    return PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory(package), 'launch', filename))


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='test_yard.world',
            description='World filename in vehicle_simulation/worlds'),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        DeclareLaunchArgument('z_pose', default_value='0.05'),

        IncludeLaunchDescription(
            _launch('vehicle_simulation', 'simulation.launch.py'),
            launch_arguments={
                'world': LaunchConfiguration('world'),
                'x_pose': LaunchConfiguration('x_pose'),
                'y_pose': LaunchConfiguration('y_pose'),
                'z_pose': LaunchConfiguration('z_pose'),
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
    ])
