#!/usr/bin/env python3
"""Start Nav2 planning/control servers for either simulation or the real car."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_NAV2_NODES = [
    ('nav2_controller', 'controller_server'),
    ('nav2_planner', 'planner_server'),
    ('nav2_behaviors', 'behavior_server'),
    ('nav2_bt_navigator', 'bt_navigator'),
    ('nav2_velocity_smoother', 'velocity_smoother'),
]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'profile', default_value='real',
            description='Configuration profile: sim or real'),
        DeclareLaunchArgument(
            'startup_delay', default_value='0.0',
            description='Seconds to wait before starting Nav2'),
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

    navigation_pkg = get_package_share_directory('vehicle_navigation')
    params = [
        os.path.join(navigation_pkg, 'config', 'nav2_params.yaml'),
        os.path.join(navigation_pkg, 'config', f'nav2_{profile}.yaml'),
    ]
    node_names = [name for _, name in _NAV2_NODES]
    nodes = [
        Node(
            package=package,
            executable=executable,
            name=executable,
            output='screen',
            parameters=params,
        )
        for package, executable in _NAV2_NODES
    ]
    nodes.append(
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[
                *params,
                {'autostart': True, 'node_names': node_names},
            ],
        ))

    return [TimerAction(period=startup_delay, actions=nodes)]
