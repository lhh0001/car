#!/usr/bin/env python3
"""Compose the physical car's Wi-Fi sensors, LiDAR and SLAM mapping.

Prerequisites:
  1. ESP32 is flashed and its ``car-esp32`` Wi-Fi AP is running.
  2. The computer is connected to that AP.
  3. The LiDAR is available either through the ESP32 TCP relay or a local
     USB serial adapter.
"""
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
            'host', default_value='192.168.4.1',
            description='ESP32 Wi-Fi address'),
        DeclareLaunchArgument(
            'lidar_transport', default_value='tcp',
            description='tcp for ESP32 relay, serial for local USB adapter'),
        DeclareLaunchArgument(
            'lidar_port_name', default_value='/dev/lidar',
            description='LiDAR serial device used when lidar_transport=serial'),

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
            _launch('vehicle_mapping', 'slam.launch.py'),
            launch_arguments={'profile': 'real', 'startup_delay': '2.0'}.items(),
        ),
    ])
