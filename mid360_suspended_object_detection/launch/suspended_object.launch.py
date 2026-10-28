from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('mid360_suspended_object_detection'),
        'config', 'detection.yaml')
    return LaunchDescription([
        Node(
            package='mid360_suspended_object_detection',
            executable='suspended_object_node',
            name='mid360_suspended_object_detection',
            output='screen',
            parameters=[params]),
    ])
