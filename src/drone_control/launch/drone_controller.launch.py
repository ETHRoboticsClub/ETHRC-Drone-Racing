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
    default_waypoints_file = PathJoinSubstitution(
        [FindPackageShare('drone_control'), "config", "waypoints.yaml"]
    )

    controller_type = LaunchConfiguration('controller_type')
    waypoints_file = LaunchConfiguration('waypoints_file')
    verbose_input = LaunchConfiguration('verbose_input')

    controller_node = Node(
        package="drone_control",
        executable="drone_controller_node",
        name="drone_controller_node",
        parameters=[controller_params, {
            'controller_type': controller_type,
            'waypoints_file': waypoints_file,
            'verbose_input': verbose_input,
        }],
        output="screen",
    )

    mavsdk_node = Node(
        package="my_mavsdk",
        executable="mavsdk_client",
        name="mavsdk_client",
        parameters=[mavsdk_client_params],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument('plotjuggler', default_value='true',
            description='Launch PlotJuggler alongside the controller'),
        DeclareLaunchArgument('controller_type', default_value='mpcc',
            description='Controller type: mpc, mpcc, mpc_rotor, position_ned, rl'),
        DeclareLaunchArgument('waypoints_file', default_value=default_waypoints_file,
            description='Path to YAML file with waypoints for the controller'),
        DeclareLaunchArgument('verbose_input', default_value='false',
            description='Print thrust and body-rate debug info'),

        # Start controller first, then mavsdk_client after 3s delay
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
