#!/usr/bin/env python3
"""
Launch diff-drive robot with Nav2 using a PRE-SAVED MAP.

Requires a map built beforehand:
  ros2 run nav2_map_server map_saver_cli -f <path>

Usage:
  ros2 launch vehicle_bringup diff_drive_nav_map.launch.py map:=path/to/map.yaml
  ros2 launch vehicle_bringup diff_drive_nav_map.launch.py map:=maps/mymap.yaml x_pose:=2.0 y_pose:=1.0
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'map', description='Path to saved map YAML (required)'),
        DeclareLaunchArgument(
            'world', default_value='world.world',
            description='Gazebo world file name'),
        DeclareLaunchArgument(
            'x_pose', default_value='0.0'),
        DeclareLaunchArgument(
            'y_pose', default_value='0.0'),
        DeclareLaunchArgument(
            'z_pose', default_value='0.05'),
        OpaqueFunction(function=_launch_setup),
    ])


def _launch_setup(context):
    desc_pkg = get_package_share_directory('vehicle_description')
    bringup_pkg = get_package_share_directory('vehicle_bringup')

    xacro_path = os.path.join(desc_pkg, 'urdf', 'diff_drive_robot.xacro')
    world_path = os.path.join(bringup_pkg, 'worlds',
                              LaunchConfiguration('world').perform(context))
    nav2_config = os.path.join(bringup_pkg, 'config', 'nav2_params.yaml')
    map_yaml = LaunchConfiguration('map').perform(context)

    import xacro
    doc = xacro.process_file(xacro_path)
    robot_desc = {'robot_description': doc.toxml()}

    x_pose = LaunchConfiguration('x_pose').perform(context)
    y_pose = LaunchConfiguration('y_pose').perform(context)
    z_pose = LaunchConfiguration('z_pose').perform(context)

    return [

        # ===== Simulation =====
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', world_path],
            output='screen',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_desc, {'use_sim_time': True}],
        ),
        TimerAction(period=2.0, actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                output='screen',
                arguments=[
                    '-entity', 'diff_drive_robot',
                    '-topic', 'robot_description',
                    '-x', x_pose, '-y', y_pose, '-z', z_pose,
                ],
            ),
        ]),

        # ===== Map Server + AMCL =====
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[nav2_config, {
                'use_sim_time': True,
                'yaml_filename': map_yaml,
            }],
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_config, {'use_sim_time': True}],
        ),

        # ===== Navigation 2 =====
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', output='screen',
             parameters=[nav2_config, {'use_sim_time': True}]),
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', output='screen',
             parameters=[nav2_config, {'use_sim_time': True}]),
        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server', output='screen',
             parameters=[nav2_config, {'use_sim_time': True}]),
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', output='screen',
             parameters=[nav2_config, {'use_sim_time': True}]),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother', output='screen',
             parameters=[nav2_config, {'use_sim_time': True}]),

        # Sequential activator: map_server → wait /map → AMCL
        TimerAction(period=4.0, actions=[
            Node(
                package='vehicle_bringup',
                executable='activate_navigation.py',
                name='nav_activator',
                output='screen',
            ),
        ]),


    ]
