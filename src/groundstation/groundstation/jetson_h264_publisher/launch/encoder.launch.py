from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="jetson_h264_publisher",
            executable="encoder_node",
            name="jetson_h264_publisher",
            output="screen",
            parameters=[{
                "sensor_id": 0,
                "width": 1920,
                "height": 1080,
                "framerate": 30,
                "bitrate": 4_000_000,
                "peak_bitrate": 6_000_000,
                "iframe_interval": 15,
                "control_rate": 1,        # 1=VBR, 2=CBR
                "preset_level": 1,        # UltraFast
                "frame_id": "camera",
                "topic": "image_raw/h264",
                "flip_method": 0,
            }],
        ),
    ])
