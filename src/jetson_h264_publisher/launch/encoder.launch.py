from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sensor_id', default_value='0'),
        DeclareLaunchArgument('width', default_value='1920'),
        DeclareLaunchArgument('height', default_value='1080'),
        DeclareLaunchArgument('framerate', default_value='30'),
        DeclareLaunchArgument('bitrate', default_value='4000000'),
        DeclareLaunchArgument('peak_bitrate', default_value='6000000'),
        DeclareLaunchArgument('iframe_interval', default_value='15'),
        DeclareLaunchArgument('control_rate', default_value='1'),
        DeclareLaunchArgument('preset_level', default_value='1'),
        DeclareLaunchArgument('flip_method', default_value='2'),
        DeclareLaunchArgument('topic', default_value='image_raw/h264'),
        DeclareLaunchArgument('frame_id', default_value='camera'),

        Node(
            package="jetson_h264_publisher",
            executable="encoder_node",
            name="jetson_h264_publisher",
            output="screen",
            parameters=[{
                "sensor_id": LaunchConfiguration('sensor_id'),
                "width": LaunchConfiguration('width'),
                "height": LaunchConfiguration('height'),
                "framerate": LaunchConfiguration('framerate'),
                "bitrate": LaunchConfiguration('bitrate'),
                "peak_bitrate": LaunchConfiguration('peak_bitrate'),
                "iframe_interval": LaunchConfiguration('iframe_interval'),
                "control_rate": LaunchConfiguration('control_rate'),
                "preset_level": LaunchConfiguration('preset_level'),
                "flip_method": LaunchConfiguration('flip_method'),
                "topic": LaunchConfiguration('topic'),
                "frame_id": LaunchConfiguration('frame_id'),
            }],
        ),
    ])
