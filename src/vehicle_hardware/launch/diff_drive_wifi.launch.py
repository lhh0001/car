#!/usr/bin/env python3
"""Launch only the ESP32 Wi-Fi hardware bridge.

Robot description publishing and teleoperation are intentionally composed by
vehicle_bringup / vehicle_control, so this launch can be reused by mapping and
navigation without creating duplicate TF publishers or /cmd_vel producers.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('host', default_value='192.168.4.1',
                              description='ESP32 wifi host'),
        DeclareLaunchArgument('port', default_value='8888',
                              description='ESP32 UDP command/telemetry port'),
        OpaqueFunction(function=_launch_setup),
    ])


def _launch_setup(context):
    hw_pkg = get_package_share_directory('vehicle_hardware')
    params = os.path.join(hw_pkg, 'config', 'motor_wifi.yaml')
    fusion_params = os.path.join(hw_pkg, 'config', 'odom_imu_fusion.yaml')
    host = LaunchConfiguration('host').perform(context)
    return [
        Node(
            package='vehicle_hardware',
            executable='wifi_bridge.py',
            name='wifi_bridge',
            output='screen',
            parameters=[params, {
                'host': host,
                'port': ParameterValue(LaunchConfiguration('port'), value_type=int),
                'odom_topic': '/wheel/odom',
                'publish_odom_tf': False,
            }],
        ),
        Node(
            package='vehicle_hardware',
            executable='odom_imu_fusion.py',
            name='odom_imu_fusion',
            output='screen',
            parameters=[fusion_params],
        ),
    ]
