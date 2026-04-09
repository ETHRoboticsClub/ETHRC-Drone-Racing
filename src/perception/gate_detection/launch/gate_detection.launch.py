from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('gate_detection')
    
    default_checkpoint = os.path.join(pkg_share, 'checkpoints', 'best_gatenet.pth')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'checkpoint',
            default_value=default_checkpoint,
            description='Path to the model checkpoint'
        ),
        DeclareLaunchArgument(
            'camera_topic',
            default_value='/cam0/image_raw',
            description='Input camera topic'
        ),
        DeclareLaunchArgument(
            'output_topic',
            default_value='/gate_detection/drone_pose',
            description='Output pose topic'
        ),
        Node(
            package='gate_detection',
            executable='detector_node',
            name='gate_detector_node',
            output='screen',
            parameters=[{
                'checkpoint': LaunchConfiguration('checkpoint'),
                'camera_topic': LaunchConfiguration('camera_topic'),
                'output_topic': LaunchConfiguration('output_topic'),
            }]
        )
    ])
