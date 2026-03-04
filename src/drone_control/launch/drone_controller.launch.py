from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
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

    controller_type = LaunchConfiguration('controller_type')

    controller_node = Node(
        package="drone_control",
        executable="drone_controller_node",
        name="drone_controller_node",
        parameters=[controller_params, {'controller_type': controller_type}],
        output="screen",
    )

    mavsdk_node = Node(
        package="my_mavsdk",
        executable="mavsdk_client",
        name="mavsdk_client",
        parameters=[mavsdk_client_params, {'controller_type': controller_type}],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument('plotjuggler', default_value='true',
            description='Launch PlotJuggler alongside the controller'),
        DeclareLaunchArgument('controller_type', default_value='mpc_rotor',
            description='Controller type: mpc, mpc_rotor, rl'),

        # Start controller first, then mavsdk_client after 3s delay
        # so the controller is ready to send commands when offboard starts
        controller_node,
        TimerAction(period=3.0, actions=[mavsdk_node]),

        Node(
            package="plotjuggler",
            executable="plotjuggler",
            output="screen",
            emulate_tty=True,
            arguments=['-l', plotjuggler_config_file],
            condition=IfCondition(LaunchConfiguration('plotjuggler')),
        ),
    ])