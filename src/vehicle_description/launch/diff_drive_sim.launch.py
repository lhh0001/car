import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch.actions import WaitForTopic



def generate_launch_description():

    pkg_dir = get_package_share_directory('vehicle_description')
    xacro_path = os.path.join(pkg_dir, 'urdf', 'diff_drive_robot.xacro')
    world_path = os.path.join(pkg_dir, 'worlds', 'test_yard.world')
    slam_config = os.path.join(pkg_dir, 'config', 'slam_mapping.yaml')

    import xacro
    doc = xacro.process_file(xacro_path)
    robot_desc = {'robot_description': doc.toxml()}

    return LaunchDescription([

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

        # Spawn robot（等 robot_description 话题上线后再 spawn）
        WaitForTopic(
            topic='/robot_description',
            timeout=10.0,
            actions=[Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_robot',
                output='screen',
                arguments=[
                    '-entity', 'diff_drive_robot',
                    '-topic', 'robot_description',
                    '-x', '0', '-y', '0', '-z', '0.05',
                ],
            )],
        ),

        # Teleop GUI
        WaitForTopic(
            topic='/cmd_vel',
            timeout=10.0,
            actions=[
                Node(
                    package='vehicle_description',
                    executable='teleop_gui.py',
                    name='teleop_gui',
                    output='screen',
                ),
            ],
        ),

        # SLAM
        WaitForTopic(
            topic='/scan',
            timeout=10.0,
            actions=[
                Node(
                    package='slam_toolbox',
                    executable='async_slam_toolbox_node',
                    name='slam_toolbox',
                    output='screen',
                    parameters=[slam_config, {'use_sim_time': True}],
                ),
            ],
        ),
    ])
