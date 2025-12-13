from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Declare world_frame argument with default value
        DeclareLaunchArgument('world_frame', default_value='world'),

        Node(
            package='Gazebo_Ground_Thruth',
            executable='Gazebo_Ground_Thruth',
            name='Gazebo_Ground_Thruth',
            output='screen',
            parameters=[{
                'world_frame': LaunchConfiguration('world_frame'),
            }]
        )
    ])
