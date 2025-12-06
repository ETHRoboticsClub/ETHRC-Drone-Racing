from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Declare world_frame argument with default value
        DeclareLaunchArgument('world_frame', default_value='world'),

        Node(
            package='pointcloud_gt',
            executable='pointcloud_GT',
            name='pointcloud_GT',
            output='screen',
            parameters=[{
                'world_frame': LaunchConfiguration('world_frame'),
            }]
        )
    ])
