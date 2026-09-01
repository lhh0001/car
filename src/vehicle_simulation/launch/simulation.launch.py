#!/usr/bin/env python3
"""Start Gazebo and spawn the differential-drive robot, without SLAM/Nav2."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='test_yard.world',
            description='World filename in vehicle_simulation/worlds, or an absolute path'),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        DeclareLaunchArgument('z_pose', default_value='0.05'),
        OpaqueFunction(function=_launch_setup),
    ])


def _launch_setup(context):
    simulation_pkg = get_package_share_directory('vehicle_simulation')
    description_pkg = get_package_share_directory('vehicle_description')
    world_value = LaunchConfiguration('world').perform(context)
    world_path = world_value if os.path.isabs(world_value) else os.path.join(
        simulation_pkg, 'worlds', world_value)
    if not os.path.isfile(world_path):
        raise RuntimeError(f'Gazebo world does not exist: {world_path}')

    simulation_model = os.path.join(
        simulation_pkg, 'urdf', 'diff_drive_robot.gazebo.xacro')
    state_publisher_launch = os.path.join(
        description_pkg, 'launch', 'state_publisher.launch.py')

    return [
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', world_path],
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(state_publisher_launch),
            launch_arguments={
                'model': simulation_model,
                'use_sim_time': 'true',
            }.items(),
        ),
        TimerAction(period=2.0, actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_robot',
                output='screen',
                arguments=[
                    '-entity', 'diff_drive_robot',
                    '-topic', 'robot_description',
                    '-x', LaunchConfiguration('x_pose'),
                    '-y', LaunchConfiguration('y_pose'),
                    '-z', LaunchConfiguration('z_pose'),
                ],
            ),
        ]),
    ]
