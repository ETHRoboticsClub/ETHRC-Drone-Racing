import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('betaflight_mavlink_bridge')
    config_file = os.path.join(pkg_share, 'config', 'params.yaml')

    return LaunchDescription([
        Node(
            package='betaflight_mavlink_bridge',
            executable='bridge_node',
            name='bridge_node',
            output='screen',
            parameters=[config_file]
        )
    ])
