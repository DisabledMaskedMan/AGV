import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package = 'mid360_suspended_object_detection'
    config = os.path.join(get_package_share_directory(package), 'config', 'scan_web.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('ws_url', default_value='ws://192.168.31.7:6060'),
        DeclareLaunchArgument('port', default_value='8765'),
        DeclareLaunchArgument('frame', default_value='scan_link'),
        DeclareLaunchArgument('config', default_value=config),
        DeclareLaunchArgument('publish_obstacle', default_value='true'),
        DeclareLaunchArgument('obstacle_topic', default_value='/mid360_suspended_object_detection/largest_obstacle'),
        Node(package=package, executable='scan_web_server', name='mid360_scan_web', output='screen',
             arguments=['--ws-url', LaunchConfiguration('ws_url'), '--port', LaunchConfiguration('port'),
                        '--host', '127.0.0.1', '--frame', LaunchConfiguration('frame'), '--config', LaunchConfiguration('config')],
             parameters=[{'publish_obstacle': ParameterValue(LaunchConfiguration('publish_obstacle'), value_type=bool),
                          'obstacle_topic': LaunchConfiguration('obstacle_topic')}]),
    ])
