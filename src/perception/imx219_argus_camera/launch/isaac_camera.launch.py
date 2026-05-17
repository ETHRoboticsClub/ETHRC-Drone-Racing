"""
IMX219 CSI camera launch for Jetson.

This uses the in-package argus_camera_node instead of gscam2 so the drone
perception stack has no external gscam runtime dependency. The node captures via
nvarguscamerasrc/libargus, drops stale frames at the appsink, and publishes:

  /cam0/image_raw
  /cam0/image_rgb
  /cam0/image_h264
  /cam0/camera_info
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('imx219_argus_camera'),
        'config', 'camera_params.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'sensor_mode', default_value='4',
            description='IMX219 sensor mode index (4 = 1280x720 high-speed mode)',
        ),
        DeclareLaunchArgument('framerate', default_value='60'),
        DeclareLaunchArgument('width', default_value='1280'),
        DeclareLaunchArgument('height', default_value='720'),
        DeclareLaunchArgument('sensor_id', default_value='0'),
        DeclareLaunchArgument('output_encoding', default_value='mono8'),
        DeclareLaunchArgument(
            'flip_method', default_value='2',
            description='nvvidconv flip-method; 2 rotates 180 degrees for the Zephyrus mount',
        ),
        DeclareLaunchArgument('publish_color', default_value='true'),
        DeclareLaunchArgument('color_encoding', default_value='rgb8'),
        DeclareLaunchArgument('color_reliable', default_value='false'),
        DeclareLaunchArgument('publish_h264', default_value='false'),
        DeclareLaunchArgument('enable_h264_stream', default_value='false'),
        DeclareLaunchArgument('ground_station_ip', default_value='127.0.0.1'),
        DeclareLaunchArgument('stream_port', default_value='5000'),
        DeclareLaunchArgument('stream_bitrate', default_value='4000000'),
        Node(
            package='imx219_argus_camera',
            executable='argus_camera_node',
            name='argus_camera_node',
            output='screen',
            parameters=[
                config_file,
                {
                    'sensor_id': ParameterValue(LaunchConfiguration('sensor_id'), value_type=int),
                    'sensor_mode': ParameterValue(LaunchConfiguration('sensor_mode'), value_type=int),
                    'framerate': ParameterValue(LaunchConfiguration('framerate'), value_type=float),
                    'width': ParameterValue(LaunchConfiguration('width'), value_type=int),
                    'height': ParameterValue(LaunchConfiguration('height'), value_type=int),
                    'output_encoding': LaunchConfiguration('output_encoding'),
                    'flip_method': ParameterValue(
                        LaunchConfiguration('flip_method'), value_type=int),
                    'publish_color': ParameterValue(
                        LaunchConfiguration('publish_color'), value_type=bool),
                    'color_encoding': LaunchConfiguration('color_encoding'),
                    'color_reliable': ParameterValue(
                        LaunchConfiguration('color_reliable'), value_type=bool),
                    'publish_h264': ParameterValue(
                        LaunchConfiguration('publish_h264'), value_type=bool),
                    'enable_h264_stream': ParameterValue(
                        LaunchConfiguration('enable_h264_stream'), value_type=bool),
                    'ground_station_ip': LaunchConfiguration('ground_station_ip'),
                    'stream_port': ParameterValue(LaunchConfiguration('stream_port'), value_type=int),
                    'stream_bitrate': ParameterValue(
                        LaunchConfiguration('stream_bitrate'), value_type=int),
                },
            ],
            remappings=[
                ('image_raw', '/cam0/image_raw'),
                ('image_rgb', '/cam0/image_rgb'),
                ('image_h264', '/cam0/image_h264'),
                ('camera_info', '/cam0/camera_info'),
            ],
        ),
    ])
