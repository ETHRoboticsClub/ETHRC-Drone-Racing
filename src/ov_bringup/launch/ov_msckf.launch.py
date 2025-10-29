from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    image_topic = LaunchConfiguration('image_topic')
    imu_topic = LaunchConfiguration('imu_topic')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value='/ws/src/ov_bringup/config/ov_params.yaml',
            description='Path to OpenVINS parameter YAML file.'
        ),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/camera/image_raw',
            description='Camera image topic to subscribe to.'
        ),
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/imu',
            description='IMU topic to subscribe to.'
        ),
        Node(
            package='ov_msckf',
            executable='ov_msckf_node',
            name='openvins',
            output='screen',
            parameters=[params_file],
            remappings=[
                ('/cam0/image_raw', image_topic),
                ('/imu0', imu_topic)
            ]
        )
    ])


