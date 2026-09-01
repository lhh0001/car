#!/usr/bin/env python3
"""Compose Gazebo simulation, SLAM and optional manual-control tools."""
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
        DeclareLaunchArgument(
            'use_gui', default_value='true',
            description='Launch the Tk manual-control window'),
        DeclareLaunchArgument(
            'monitor', default_value='false',
            description='Launch the optional PyQt ROS graph monitor'),

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
            _launch('vehicle_control', 'manual_control.launch.py'),
            launch_arguments={
                'teleop': LaunchConfiguration('use_gui'),
                'monitor': LaunchConfiguration('monitor'),
            }.items(),
        ),
    ])
