#!/usr/bin/env python3
"""Load a saved map and start AMCL for either simulation or the real car."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'map', description='Absolute path to the saved map YAML file'),
        DeclareLaunchArgument(
            'profile', default_value='real',
            description='Configuration profile: sim or real'),
        DeclareLaunchArgument(
            'startup_delay', default_value='0.0',
            description='Seconds to wait before starting map_server and AMCL'),
        OpaqueFunction(function=_launch_setup),
    ])


def _launch_setup(context):
    map_path = LaunchConfiguration('map').perform(context)
    if not map_path or not os.path.isfile(map_path):
        raise RuntimeError(f'Saved map does not exist: {map_path}')

    profile = LaunchConfiguration('profile').perform(context)
    if profile not in ('sim', 'real'):
        raise RuntimeError("profile must be either 'sim' or 'real'")
    try:
        startup_delay = float(LaunchConfiguration('startup_delay').perform(context))
    except ValueError as exc:
        raise RuntimeError('startup_delay must be a number') from exc

    navigation_pkg = get_package_share_directory('vehicle_navigation')
    params = [
        os.path.join(navigation_pkg, 'config', 'nav2_params.yaml'),
        os.path.join(navigation_pkg, 'config', f'nav2_{profile}.yaml'),
    ]
    nodes = [
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[*params, {'yaml_filename': map_path}],
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=params,
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[
                *params,
                {'autostart': True, 'node_names': ['map_server', 'amcl']},
            ],
        ),
    ]
    return [TimerAction(period=startup_delay, actions=nodes)]
