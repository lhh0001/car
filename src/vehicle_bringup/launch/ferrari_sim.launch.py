#!/usr/bin/env python3
"""
Launch Ferrari 4-wheel Ackermann vehicle simulation.

Usage:
  ros2 launch vehicle_bringup ferrari_sim.launch.py
  ros2 launch vehicle_bringup ferrari_sim.launch.py world:=city_block.world
  ros2 launch vehicle_bringup ferrari_sim.launch.py use_slam:=true
  ros2 launch vehicle_bringup ferrari_sim.launch.py x_pose:=5.0 y_pose:=3.0
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
            'world', default_value='test_yard.world',
            description='Gazebo world file (in vehicle_bringup/worlds/)'),
        DeclareLaunchArgument(
            'x_pose', default_value='0.0',
            description='Initial vehicle X position'),
        DeclareLaunchArgument(
            'y_pose', default_value='0.0',
            description='Initial vehicle Y position'),
        DeclareLaunchArgument(
            'z_pose', default_value='0.3',
            description='Initial vehicle Z position'),
        DeclareLaunchArgument(
            'use_slam', default_value='true',
            description='Launch SLAM Toolbox for mapping'),
        OpaqueFunction(function=_launch_setup),
    ])


def _launch_setup(context):
    desc_pkg = get_package_share_directory('vehicle_description')
    bringup_pkg = get_package_share_directory('vehicle_bringup')

    urdf_path = os.path.join(desc_pkg, 'urdf', 'ferrari.urdf')
    ctrl_path = os.path.join(bringup_pkg, 'config', 'steering_controller.yaml')
    world_path = os.path.join(bringup_pkg, 'worlds', LaunchConfiguration('world').perform(context))
    slam_config = os.path.join(bringup_pkg, 'config', 'slam_mapping.yaml')

    with open(urdf_path, 'r') as f:
        robot_desc_text = f.read()

    # Expand all placeholder paths in the URDF:
    #   package://pkg_name/...  → file:///absolute/...   (for mesh paths, Gazebo can read)
    #   @pkg_name_share@/...    → /absolute/...           (for plugin parameters, needs plain path)
    robot_desc_text = _expand_urdf_paths(robot_desc_text)

    robot_desc = {'robot_description': robot_desc_text}

    x_pose = LaunchConfiguration('x_pose').perform(context)
    y_pose = LaunchConfiguration('y_pose').perform(context)
    z_pose = LaunchConfiguration('z_pose').perform(context)
    use_slam = LaunchConfiguration('use_slam').perform(context)

    # Gazebo needs GAZEBO_PLUGIN_PATH to find ROS2 plugins
    gazebo_env = dict(os.environ)
    gazebo_env['GAZEBO_PLUGIN_PATH'] = '/opt/ros/humble/lib'

    actions = [

        # Gazebo with custom world
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', world_path],
            output='screen',
            env=gazebo_env,
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_desc, {'use_sim_time': True}],
        ),

        # ros2_control_node (must start before model spawn)
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            name='controller_manager',
            parameters=[robot_desc, ctrl_path],
            output='screen',
        ),

        # TF bridge: base_footprint → body_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_body_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'body_link'],
        ),

        # Spawn vehicle in Gazebo
        TimerAction(period=2.0, actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_vehicle',
                output='screen',
                arguments=[
                    '-entity', 'vehicle',
                    '-topic', 'robot_description',
                    '-x', x_pose, '-y', y_pose, '-z', z_pose,
                ],
            ),
        ]),

        # Spawn joint_state_broadcaster
        TimerAction(period=4.0, actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster'],
                output='screen',
            ),
        ]),

        # Spawn steering controller
        TimerAction(period=5.0, actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['steering_controller'],
                output='screen',
            ),
        ]),

        # Steering logic node (from vehicle_control package)
        TimerAction(period=6.0, actions=[
            Node(
                package='vehicle_control',
                executable='steering_controller.py',
                name='steering_controller_node',
                output='screen',
            ),
        ]),
    ]

    # Optional SLAM
    if use_slam.lower() in ('true', '1'):
        actions.append(
            TimerAction(period=8.0, actions=[
                Node(
                    package='slam_toolbox',
                    executable='async_slam_toolbox_node',
                    name='slam_toolbox',
                    output='screen',
                    parameters=[slam_config, {'use_sim_time': True}],
                ),
            ]),
        )

    return actions


def _expand_urdf_paths(urdf_text: str) -> str:
    """Replace portable placeholders with absolute paths for Gazebo.

    Two placeholder formats:
      package://pkg/...  → file:///install/path/...   (mesh URIs)
      @pkg_share@/...    → /install/path/...           (plain filesystem paths)

    Gazebo Classic needs file:// for meshes, but plain paths for plugin
    parameters (gazebo_ros2_control opens them directly).
    """
    import re
    from ament_index_python.packages import get_package_share_directory

    def _resolve(pkg_name: str) -> str:
        """Look up a package's share directory, returning empty string on failure."""
        try:
            return get_package_share_directory(pkg_name)
        except Exception:
            return ''

    # 1) package://pkg_name/... → file://... (for mesh/visual paths)
    def _expand_package(m):
        share = _resolve(m.group(1))
        return f'file://{share}/' if share else m.group(0)

    urdf_text = re.sub(r'package://([a-zA-Z_][\w]*)/', _expand_package, urdf_text)

    # 2) @pkg_name_share@/... → /absolute/... (for plugin parameters — NO file://)
    def _expand_at(m):
        share = _resolve(m.group(1))
        return f'{share}/' if share else m.group(0)

    urdf_text = re.sub(r'@([a-zA-Z_][\w]*)_share@/', _expand_at, urdf_text)

    return urdf_text
