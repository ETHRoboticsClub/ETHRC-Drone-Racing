from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sensor_id', default_value='0'),
        DeclareLaunchArgument('sensor_mode', default_value='4'),
        DeclareLaunchArgument('width', default_value='1280'),
        DeclareLaunchArgument('height', default_value='720'),
        DeclareLaunchArgument('framerate', default_value='60'),
        DeclareLaunchArgument('bitrate', default_value='4000000'),
        DeclareLaunchArgument('iframe_interval', default_value='30'),
        DeclareLaunchArgument('control_rate', default_value='1'),
        DeclareLaunchArgument('preset_level', default_value='0'),
        DeclareLaunchArgument('ground_station_ip', default_value='127.0.0.1'),
        DeclareLaunchArgument('port', default_value='5000'),
        DeclareLaunchArgument('payload_type', default_value='96'),
        DeclareLaunchArgument('encoder_options', default_value='tune=zerolatency'),

        Node(
            package="jetson_h264_publisher",
            executable="encoder_node",
            name="jetson_h264_streamer",
            output="screen",
            parameters=[{
                "sensor_id": LaunchConfiguration('sensor_id'),
                "sensor_mode": LaunchConfiguration('sensor_mode'),
                "width": LaunchConfiguration('width'),
                "height": LaunchConfiguration('height'),
                "framerate": LaunchConfiguration('framerate'),
                "bitrate": LaunchConfiguration('bitrate'),
                "iframe_interval": LaunchConfiguration('iframe_interval'),
                "control_rate": LaunchConfiguration('control_rate'),
                "preset_level": LaunchConfiguration('preset_level'),
                "ground_station_ip": LaunchConfiguration('ground_station_ip'),
                "port": LaunchConfiguration('port'),
                "payload_type": LaunchConfiguration('payload_type'),
                "encoder_options": LaunchConfiguration('encoder_options'),
            }],
        ),
    ])
