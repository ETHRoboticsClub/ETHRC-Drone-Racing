from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    plotjuggler_config_file = PathJoinSubstitution(
        [FindPackageShare('drone_control'), "config", "control_plotjuggler.xml"]
    )
    controller_params = PathJoinSubstitution(
        [FindPackageShare('drone_control'), "config", "drone_controller_config.yaml"]
    )
    mavsdk_client_params = PathJoinSubstitution(
        [FindPackageShare('my_mavsdk'), "config", "mavsdk_client_config.yaml"]
    )

    return LaunchDescription([
        DeclareLaunchArgument('plotjuggler', default_value='true',
            description='Launch PlotJuggler alongside the controller'),
        DeclareLaunchArgument('use_mavsdk', default_value='true',
            description='Whether to launch the MAVSDK client'),
        DeclareLaunchArgument('controller_type', default_value='rl',
            description='Type of controller to use: "mpc", "rl", or "random"'),
        DeclareLaunchArgument('rate_error_scale', default_value='300.0',
            description='Torque-to-rate scaling for RL controller'),

        Node(
            package="my_mavsdk",
            executable="mavsdk_client",
            name="mavsdk_client",
            parameters=[mavsdk_client_params],
            output="screen",
            condition=IfCondition(LaunchConfiguration('use_mavsdk')),
        ),
        Node(
            package="drone_control",
            executable="drone_controller_node",
            name="drone_controller_node",
            parameters=[controller_params, {
                'controller_type': LaunchConfiguration('controller_type'),
                'rl.rate_error_scale': LaunchConfiguration('rate_error_scale')
            }],
            output="screen",
        ),
        Node(
            package="plotjuggler",
            executable="plotjuggler",
            output="screen",
            emulate_tty=True,
            arguments=['-l', plotjuggler_config_file],
            condition=IfCondition(LaunchConfiguration('plotjuggler')),
        ),
    ])