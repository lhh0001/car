#!/usr/bin/env python3
"""Publish the robot description and its fixed/joint TF frames.

The default model is the physical-car description.  Simulation passes its
Gazebo overlay through the ``model`` argument, so this launch stays reusable
without embedding simulator code in the description package.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_pkg = get_package_share_directory('vehicle_description')
    default_model = os.path.join(description_pkg, 'urdf', 'diff_drive_robot.xacro')

    return LaunchDescription([
        DeclareLaunchArgument(
            'model', default_value=default_model,
            description='Absolute path to the Xacro model to publish'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use Gazebo /clock when true'),
        DeclareLaunchArgument(
            'wheel_radius', default_value='0.02',
            description='Wheel radius passed to the Xacro model'),
        DeclareLaunchArgument(
            'wheel_separation', default_value='0.13',
            description='Distance between left and right wheel centers'),
        DeclareLaunchArgument('wheel_width', default_value='0.015'),
        DeclareLaunchArgument('body_length', default_value='0.20'),
        DeclareLaunchArgument('body_width', default_value='0.13'),
        DeclareLaunchArgument(
            'lidar_yaw', default_value='-0.654498469',
            description='Physical LiDAR yaw relative to base_link, radians'),
        OpaqueFunction(function=_launch_setup),
    ])


def _launch_setup(context):
    model_path = LaunchConfiguration('model').perform(context)
    if not os.path.isfile(model_path):
        raise RuntimeError(f'Robot model does not exist: {model_path}')

    import xacro

    robot_description = {
        'robot_description': xacro.process_file(
            model_path,
            mappings={
                'wheel_radius': LaunchConfiguration('wheel_radius').perform(context),
                'wheel_separation': LaunchConfiguration('wheel_separation').perform(context),
                'wheel_width': LaunchConfiguration('wheel_width').perform(context),
                'body_length': LaunchConfiguration('body_length').perform(context),
                'body_width': LaunchConfiguration('body_width').perform(context),
                'lidar_yaw': LaunchConfiguration('lidar_yaw').perform(context),
            },
        ).toxml()
    }
    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                robot_description,
                {
                    'use_sim_time': ParameterValue(
                        LaunchConfiguration('use_sim_time'), value_type=bool),
                },
            ],
        ),
    ]
