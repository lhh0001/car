#!/usr/bin/env python3
"""
Launch diff-drive robot simulation with SLAM.

Usage:
  ros2 launch vehicle_bringup diff_drive_sim.launch.py
  ros2 launch vehicle_bringup diff_drive_sim.launch.py world:=my_world.world
  ros2 launch vehicle_bringup diff_drive_sim.launch.py x_pose:=2.0 y_pose:=1.0
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ---- packages ----
    desc_pkg = get_package_share_directory('vehicle_description')
    bringup_pkg = get_package_share_directory('vehicle_bringup')

    # ---- launch arguments ----
    world_arg = DeclareLaunchArgument(
        'world', default_value='test_yard.world',
        description='Gazebo world file name (looked up in vehicle_bringup/worlds/)')
    x_pose_arg = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='Initial robot X position')
    y_pose_arg = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Initial robot Y position')
    z_pose_arg = DeclareLaunchArgument(
        'z_pose', default_value='0.05',
        description='Initial robot Z position')
    use_gui_arg = DeclareLaunchArgument(
        'use_gui', default_value='true',
        description='Launch teleop GUI')
    monitor_arg = DeclareLaunchArgument(
        'monitor', default_value='false',
        description='Launch graph monitor (needs PyQt5)')

    return LaunchDescription([
        world_arg,
        x_pose_arg,
        y_pose_arg,
        z_pose_arg,
        use_gui_arg,
        monitor_arg,
        OpaqueFunction(function=_launch_setup),
    ])


def _launch_setup(context):
    desc_pkg = get_package_share_directory('vehicle_description')
    bringup_pkg = get_package_share_directory('vehicle_bringup')

    xacro_path = os.path.join(desc_pkg, 'urdf', 'diff_drive_robot.xacro')
    world_path = os.path.join(bringup_pkg, 'worlds', LaunchConfiguration('world').perform(context))
    slam_config = os.path.join(bringup_pkg, 'config', 'slam_mapping.yaml')

    import xacro
    doc = xacro.process_file(xacro_path)
    robot_desc = {'robot_description': doc.toxml()}

    x_pose = LaunchConfiguration('x_pose').perform(context)
    y_pose = LaunchConfiguration('y_pose').perform(context)
    z_pose = LaunchConfiguration('z_pose').perform(context)
    use_gui = LaunchConfiguration('use_gui').perform(context)
    use_monitor = LaunchConfiguration('monitor').perform(context)

    actions = [
        # Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', world_path],
            output='screen',
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_desc, {'use_sim_time': True}],
        ),

        # Spawn robot (delay for robot_description to be published)
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

        # SLAM (delay for LiDAR to start publishing /scan)
        TimerAction(period=4.0, actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[slam_config, {'use_sim_time': True}],
            ),
        ]),
    ]

    # Optional: Teleop GUI
    if use_gui.lower() in ('true', '1'):
        actions.append(
            TimerAction(period=5.0, actions=[
                Node(
                    package='vehicle_control',
                    executable='teleop_gui.py',
                    name='teleop_gui',
                    output='screen',
                ),
            ]),
        )

    # Optional: Graph Monitor (needs PyQt5)
    if use_monitor.lower() in ('true', '1'):
        actions.append(
            TimerAction(period=3.0, actions=[
                Node(
                    package='vehicle_control',
                    executable='graph_monitor.py',
                    name='graph_monitor',
                    output='screen',
                ),
            ]),
        )

    return actions
