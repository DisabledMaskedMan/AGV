"""Start MID360 detection, optionally with the interactive RViz ROI editor."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_nodes(context):
    package_share = get_package_share_directory('mid360_obstacle_avoidance')
    # Save to the source configuration path, not launch's temporary parameter file.
    params_file = str(Path(LaunchConfiguration('params_file').perform(context)).expanduser().absolute())
    return [
        Node(
            package='livox_ros_driver2',
            executable='livox_ros_driver2_node',
            name='livox_lidar_publisher',
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_driver')),
            parameters=[{
                'xfer_format': 1,
                'multi_topic': 0,
                'data_src': 0,
                'publish_freq': 10.0,
                'output_data_type': 0,
                'frame_id': ParameterValue(LaunchConfiguration('frame_id'), value_type=str),
                'user_config_path': ParameterValue(
                    LaunchConfiguration('lidar_config'), value_type=str),
                'cmdline_input_bd_code': 'livox0000000001',
            }],
            remappings=[('/livox/lidar', LaunchConfiguration('input_topic'))],
        ),
        Node(
            package='mid360_obstacle_avoidance',
            executable='mid360_obstacle_node',
            name='mid360_obstacle_avoidance',
            output='screen',
            arguments=['--ros-args', '--log-level', LaunchConfiguration('detector_log_level')],
            parameters=[params_file, {
                'input_topic': ParameterValue(LaunchConfiguration('input_topic'), value_type=str),
                'expected_frame': ParameterValue(LaunchConfiguration('frame_id'), value_type=str),
                'enable_roi_editor': ParameterValue(
                    LaunchConfiguration('enable_roi_editor'), value_type=bool),
                'enable_web_editor': ParameterValue(
                    LaunchConfiguration('enable_web_editor'), value_type=bool),
                'roi_config_file': params_file,
            }],
        ),
        Node(
            package='mid360_obstacle_avoidance',
            executable='mid360_web_editor.py', name='mid360_web_editor', output='screen',
            condition=IfCondition(LaunchConfiguration('enable_web_editor')),
            parameters=[{
                'web_host': ParameterValue(LaunchConfiguration('web_host'), value_type=str),
                'web_port': ParameterValue(LaunchConfiguration('web_port'), value_type=int),
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='mid360_roi_rviz',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_roi_editor')),
            arguments=['-d', package_share + '/rviz/mid360_roi_editor.rviz',
                       '-f', LaunchConfiguration('frame_id')],
        ),
    ]


def generate_launch_description():
    package_share = get_package_share_directory('mid360_obstacle_avoidance')
    driver_share = get_package_share_directory('livox_ros_driver2')
    return LaunchDescription([
        DeclareLaunchArgument('start_driver', default_value='true',
                              description='Set false when a driver or rosbag is already running.'),
        DeclareLaunchArgument('params_file',
                              default_value=package_share + '/config/obstacle.yaml'),
        DeclareLaunchArgument('lidar_config',
                              default_value=driver_share + '/config/MID360_config.json'),
        DeclareLaunchArgument('input_topic', default_value='/livox/lidar'),
        DeclareLaunchArgument('frame_id', default_value='livox_frame',
                              description='Frame label only; does not transform the point cloud.'),
        DeclareLaunchArgument('enable_roi_editor', default_value='false',
                              description='Start RViz and enable live ROI drag/save controls.'),
        DeclareLaunchArgument('enable_web_editor', default_value='false'),
        DeclareLaunchArgument('web_host', default_value='127.0.0.1'),
        DeclareLaunchArgument('web_port', default_value='8080'),
        DeclareLaunchArgument('detector_log_level', default_value='info'),
        OpaqueFunction(function=launch_nodes),
    ])
