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

    return LaunchDescription([
        DeclareLaunchArgument('plotjuggler', default_value='false',
            description='Launch PlotJuggler alongside the controller'),

        Node(
            package="drone_control",
            executable="drone_controller_node",
            name="drone_controller_node",
            parameters=[controller_params],
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