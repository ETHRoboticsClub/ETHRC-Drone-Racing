from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port_arg = DeclareLaunchArgument('port', default_value='8080',
                                     description='HTTP port for the web UI')
    host_arg = DeclareLaunchArgument('host', default_value='0.0.0.0',
                                     description='Bind address (0.0.0.0 exposes on LAN)')
    topic_arg = DeclareLaunchArgument('odometry_topic', default_value='/ekf/odometry',
                                      description='Odometry topic to subscribe to')

    return LaunchDescription([
        port_arg, host_arg, topic_arg,
        Node(
            package='groundstation',
            executable='web_server',
            name='groundstation_web_server',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'host': LaunchConfiguration('host'),
                'odometry_topic': LaunchConfiguration('odometry_topic'),
            }],
        ),
    ])
