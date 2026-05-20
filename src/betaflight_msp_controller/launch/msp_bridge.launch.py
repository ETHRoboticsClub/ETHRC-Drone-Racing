import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # Package Directory
    pkg_share = get_package_share_directory('betaflight_msp_controller')

    # Path to the params file
    default_params_file = os.path.join(pkg_share, 'config', 'msp_params.yaml')

    # Declare Launch Arguments
    # These match the parameters declared in the ROS 2 node
    declare_port_cmd = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyTHS1',
        description='Serial port for MSP communication'
    )

    declare_baudrate_cmd = DeclareLaunchArgument(
        'baudrate',
        default_value='1500000',
        description='Baudrate for serial communication'
    )

    declare_att_topic_cmd = DeclareLaunchArgument(
        'attitude_cmd_topic',
        default_value='/drone_attitude_cmd',
        description='Topic for incoming attitude commands'
    )

    # MSP Bridge Node
    msp_bridge_node = Node(
        package='betaflight_msp_controller',
        executable='msp_bridge_node',
        name='msp_bridge_node',
        output='screen',
        parameters=[
            default_params_file,
            {
                'port': LaunchConfiguration('port'),
                'baudrate': LaunchConfiguration('baudrate'),
                'attitude_cmd_topic': LaunchConfiguration('attitude_cmd_topic'),
            }
        ]
    )

    return LaunchDescription([
        declare_port_cmd,
        declare_baudrate_cmd,
        declare_att_topic_cmd,
        msp_bridge_node
    ])
