from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    default_waypoints_file = PathJoinSubstitution(
        [FindPackageShare('drone_control'), 'config', 'waypoints_config.yaml']
    )
    rviz_config = PathJoinSubstitution(
        [FindPackageShare('drone_control'), 'rviz', 'drone_racing.rviz']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'waypoints_file',
            default_value=default_waypoints_file,
            description='Path to YAML file with waypoints to visualize',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz2 alongside the visualizer node',
        ),

        Node(
            package='drone_control',
            executable='waypoint_visualizer',
            name='waypoint_visualizer',
            parameters=[{'waypoints_file': LaunchConfiguration('waypoints_file')}],
            output='screen',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='screen',
        ),
    ])
