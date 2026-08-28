#!/usr/bin/env python3
"""
Launch real diff-drive robot (ESP32 hardware).

Usage:
  ros2 launch vehicle_hardware diff_drive_real.launch.py
  ros2 launch vehicle_hardware diff_drive_real.launch.py port:=/dev/ttyUSB1
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/esp32',
                              description='ESP32 serial port'),
        DeclareLaunchArgument('baud', default_value='115200',
                              description='Serial baud rate'),
        OpaqueFunction(function=_launch_setup),
    ])


def _launch_setup(context):
    desc_pkg = get_package_share_directory('vehicle_description')
    hw_pkg = get_package_share_directory('vehicle_hardware')
    params = os.path.join(hw_pkg, 'config', 'motor_params.yaml')
    port = LaunchConfiguration('port').perform(context)
    baud = LaunchConfiguration('baud').perform(context)

    # xacro → URDF
    import xacro
    xacro_path = os.path.join(desc_pkg, 'urdf', 'diff_drive_robot.xacro')
    doc = xacro.process_file(xacro_path)
    robot_desc = {'robot_description': doc.toxml()}

    return [
        # Serial bridge: PC ↔ ESP32
        Node(
            package='vehicle_hardware',
            executable='serial_bridge.py',
            name='serial_bridge',
            output='screen',
            parameters=[params, {'port': port, 'baud': int(baud)}],
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_desc],
        ),

        # Teleop GUI
        Node(
            package='vehicle_control',
            executable='teleop_gui.py',
            name='teleop_gui',
            output='screen',
        ),
    ]
