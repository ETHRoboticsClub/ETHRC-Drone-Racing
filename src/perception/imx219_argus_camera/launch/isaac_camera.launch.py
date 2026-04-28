"""
GStreamer camera pipeline for IMX219 CSI on Jetson (via gscam2).

Uses nvarguscamerasrc (GPU-accelerated capture via libargus) with
nvvidconv for hardware color conversion, wrapped by the gscam2 ROS 2
driver.  Publishes ``/cam0/image_raw`` (rgb8) and ``/cam0/camera_info``
for downstream CPU consumers (OpenVINS, ArUco, rosbag).

IMX219 sensor modes (from device tree on Jetson Orin NX):
    mode 0 : 3280x2464 @ 21 fps
    mode 1 : 3280x1848 @ 28 fps
    mode 2 : 1920x1080 @ 30 fps
    mode 3 : 1640x1232 @ 30 fps
    mode 4 : 1280x720  @ 90 fps   <-- default for racing (with 90Hz driver mod)
                                       (60 fps stock)

Usage:
    ros2 launch imx219_argus_camera isaac_camera.launch.py
    ros2 launch imx219_argus_camera isaac_camera.launch.py sensor_mode:=2 framerate:=30
    ros2 launch imx219_argus_camera isaac_camera.launch.py framerate:=60  # stock driver
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context):
    sensor_mode = LaunchConfiguration('sensor_mode').perform(context)
    framerate = LaunchConfiguration('framerate').perform(context)
    width = LaunchConfiguration('width').perform(context)
    height = LaunchConfiguration('height').perform(context)
    sensor_id = LaunchConfiguration('sensor_id').perform(context)

    gscam_config = (
        f'nvarguscamerasrc sensor-id={sensor_id} sensor-mode={sensor_mode} '
        f'exposuretimerange="1000000 5000000" gainrange="1 8" '
        f'! video/x-raw(memory:NVMM),width={width},height={height},'
        f'framerate={framerate}/1 '
        f'! nvvidconv '
        f'! video/x-raw,format=BGRx '
        f'! videoconvert '
        f'! video/x-raw,format=RGB'
    )

    config_file = os.path.join(
        get_package_share_directory('imx219_argus_camera'),
        'config', 'gscam_imx219.yaml',
    )

    camera_node = Node(
        package='gscam2',
        executable='gscam_main',
        name='gscam_publisher',
        output='screen',
        parameters=[
            config_file,
            {'gscam_config': gscam_config},
        ],
        remappings=[
            ('image_raw', '/cam0/image_raw'),
            ('camera_info', '/cam0/camera_info'),
        ],
    )

    return [camera_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'sensor_mode', default_value='4',
            description='IMX219 sensor mode index (4 = 1280x720@90fps w/ mod)',
        ),
        DeclareLaunchArgument(
            'framerate', default_value='90',
            description='Target FPS (requires 90Hz driver mod for >60)',
        ),
        DeclareLaunchArgument('width', default_value='1280'),
        DeclareLaunchArgument('height', default_value='720'),
        DeclareLaunchArgument('sensor_id', default_value='0'),
        OpaqueFunction(function=_launch_setup),
    ])
