#!/usr/bin/env python3
"""Start slam_toolbox using the project's shared mapping configuration."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'profile', default_value='real',
            description='Configuration profile: sim or real'),
        DeclareLaunchArgument(
            'startup_delay', default_value='2.0',
            description='Seconds to wait for /scan and odometry TF'),
        OpaqueFunction(function=_launch_setup),
    ])


def _launch_setup(context):
    profile = LaunchConfiguration('profile').perform(context)
    if profile not in ('sim', 'real'):
        raise RuntimeError("profile must be either 'sim' or 'real'")

    try:
        startup_delay = float(LaunchConfiguration('startup_delay').perform(context))
    except ValueError as exc:
        raise RuntimeError('startup_delay must be a number') from exc

    mapping_pkg = get_package_share_directory('vehicle_mapping')
    common_config = os.path.join(mapping_pkg, 'config', 'slam_mapping.yaml')
    profile_config = os.path.join(mapping_pkg, 'config', f'slam_{profile}.yaml')

    return [
        TimerAction(period=startup_delay, actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[common_config, profile_config],
            ),
        ]),
    ]
