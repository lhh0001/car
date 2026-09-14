#!/usr/bin/env python3
"""Start the physical robot on a saved map with AMCL, Nav2 and RViz."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
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
    mapping_share = get_package_share_directory('vehicle_mapping')
    default_map = os.path.join(
        mapping_share, 'maps', 'real_navigation.yaml')
    rviz_config = os.path.join(
        mapping_share, 'config', 'real_mapping.rviz')

    return LaunchDescription([
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('host', default_value='192.168.4.1'),
        DeclareLaunchArgument('lidar_transport', default_value='serial'),
        DeclareLaunchArgument('lidar_port_name', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('rviz', default_value='true'),

        IncludeLaunchDescription(
            _launch('vehicle_description', 'state_publisher.launch.py'),
            launch_arguments={'use_sim_time': 'false'}.items(),
        ),
        IncludeLaunchDescription(
            _launch('vehicle_hardware', 'diff_drive_wifi.launch.py'),
            launch_arguments={'host': LaunchConfiguration('host')}.items(),
        ),
        IncludeLaunchDescription(
            _launch('lidar_pkg', 'lidar.launch.py'),
            launch_arguments={
                'transport': LaunchConfiguration('lidar_transport'),
                'tcp_host': LaunchConfiguration('host'),
                'tcp_port': '8889',
                'port_name': LaunchConfiguration('lidar_port_name'),
            }.items(),
        ),
        IncludeLaunchDescription(
            _launch('vehicle_navigation', 'localization.launch.py'),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'profile': 'real',
                'startup_delay': '2.0',
            }.items(),
        ),
        IncludeLaunchDescription(
            _launch('vehicle_navigation', 'navigation.launch.py'),
            launch_arguments={
                'profile': 'real',
                'startup_delay': '4.0',
            }.items(),
        ),
        TimerAction(period=5.0, actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz',
                output='screen',
                arguments=['-d', rviz_config],
                condition=IfCondition(LaunchConfiguration('rviz')),
            ),
        ]),
    ])
