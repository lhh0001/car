#!/usr/bin/env python3
"""
Launch diff-drive robot with SLAM + Nav2 full autonomous navigation.

Usage:
  ros2 launch vehicle_bringup diff_drive_nav.launch.py
  ros2 launch vehicle_bringup diff_drive_nav.launch.py world:=warehouse.world
  ros2 launch vehicle_bringup diff_drive_nav.launch.py x_pose:=3.0 y_pose:=-2.0
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _make_nav2_nodes(config):
    """Create all Nav2 server nodes with a common config."""
    nodes = [
        ('nav2_controller', 'controller_server'),
        ('nav2_planner', 'planner_server'),
        ('nav2_behaviors', 'behavior_server'),
        ('nav2_bt_navigator', 'bt_navigator'),
        ('nav2_velocity_smoother', 'velocity_smoother'),
    ]
    return [
        Node(package=pkg, executable=exe, name=exe, output='screen',
             parameters=[config, {'use_sim_time': True}])
        for pkg, exe in nodes
    ]


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='test_yard.world',
            description='Gazebo world file name (looked up in vehicle_bringup/worlds/)'),
        DeclareLaunchArgument(
            'x_pose', default_value='0.0',
            description='Initial robot X position'),
        DeclareLaunchArgument(
            'y_pose', default_value='0.0',
            description='Initial robot Y position'),
        DeclareLaunchArgument(
            'z_pose', default_value='0.05',
            description='Initial robot Z position'),
        OpaqueFunction(function=_launch_setup),
    ])


def _launch_setup(context):
    desc_pkg = get_package_share_directory('vehicle_description')
    bringup_pkg = get_package_share_directory('vehicle_bringup')

    xacro_path = os.path.join(desc_pkg, 'urdf', 'diff_drive_robot.xacro')
    world_path = os.path.join(bringup_pkg, 'worlds', LaunchConfiguration('world').perform(context))
    slam_config = os.path.join(bringup_pkg, 'config', 'slam_mapping.yaml')
    nav2_config = os.path.join(bringup_pkg, 'config', 'nav2_params.yaml')

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
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_desc, {'use_sim_time': True}],
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
                    '-x', x_pose, '-y', y_pose, '-z', z_pose,
                ],
            ),
        ]),

        # ===== SLAM =====
        TimerAction(period=6.0, actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[slam_config, {'use_sim_time': True}],
            ),
        ]),

        # ===== Navigation 2 =====
        *_make_nav2_nodes(nav2_config),

        # Lifecycle manager: auto-activates all Nav2 nodes
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[nav2_config, {
                'use_sim_time': True,
                'autostart': True,
                'node_names': [
                    'controller_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                    'velocity_smoother',
                ],
            }],
        ),
    ]
