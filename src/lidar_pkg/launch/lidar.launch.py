import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('lidar_pkg'),
        'config',
        'lidar_params.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'transport', default_value='tcp',
            description='tcp for the ESP32 relay, serial for a USB-TTL adapter'),
        DeclareLaunchArgument('tcp_host', default_value='192.168.4.1'),
        DeclareLaunchArgument('tcp_port', default_value='8889'),
        DeclareLaunchArgument('port_name', default_value='/dev/lidar'),
        Node(
            package='lidar_pkg',
            executable='lidar_node',
            name='lidar_node',
            output='screen',
            parameters=[config_path, {
                'transport': LaunchConfiguration('transport'),
                'tcp_host': LaunchConfiguration('tcp_host'),
                'tcp_port': ParameterValue(
                    LaunchConfiguration('tcp_port'), value_type=int),
                'port_name': LaunchConfiguration('port_name'),
            }],
        ),
    ])
