from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
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
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('drone_control'), "rviz", "drone_racing.rviz"]
    )

    controller_type = LaunchConfiguration('controller_type')
    waypoints_file = LaunchConfiguration('waypoints_file')
    verbose_input = LaunchConfiguration('verbose_input')

    def create_nodes(context, *args, **kwargs):
        controller_param_list = [controller_params]
        if context.launch_configurations.get('controller_type', ''):
            controller_param_list.append({'controller_type': context.launch_configurations['controller_type']})
        if context.launch_configurations.get('waypoints_file', ''):
            controller_param_list.append({'waypoints_file': context.launch_configurations['waypoints_file']})
        if context.launch_configurations.get('verbose_input', ''):
            verbose_val = context.launch_configurations['verbose_input'].lower()
            controller_param_list.append(
                {'verbose_input': verbose_val in ('true', '1', 'yes', 'y')}
            )

        controller_node = Node(
            package="drone_control",
            executable="drone_controller_node",
            name="drone_controller_node",
            parameters=controller_param_list,
            output="screen",
        )

        mavsdk_node = Node(
            package="my_mavsdk",
            executable="mavsdk_client",
            name="mavsdk_client",
            parameters=[mavsdk_client_params],
            output="screen",
        )

        visualizer_node = Node(
            package="drone_control",
            executable="waypoint_visualizer",
            name="waypoint_visualizer",
            parameters=[{'waypoints_file': context.launch_configurations.get('waypoints_file', '')}],
            output="screen",
        )

        return [
            controller_node,
            visualizer_node,
            TimerAction(period=3.0, actions=[mavsdk_node]),
        ]

    return LaunchDescription([
        DeclareLaunchArgument('plotjuggler', default_value='true',
            description='Launch PlotJuggler alongside the controller'),
        DeclareLaunchArgument('rviz', default_value='true',
            description='Launch RViz2 alongside the controller'),
        DeclareLaunchArgument('controller_type', default_value='',
            description='Controller type: mpc, mpcc, mpcc_pp, mpc_rotor, position_ned, rl; leave empty to use config file default'),
        DeclareLaunchArgument('waypoints_file', default_value='',
            description='Path to YAML file with waypoints for the controller; leave empty to disable default trajectory'),
        DeclareLaunchArgument('verbose_input', default_value='false',
            description='Print thrust and body-rate debug info'),

        OpaqueFunction(function=create_nodes),

        Node(
            package="plotjuggler",
            executable="plotjuggler",
            output="screen",
            emulate_tty=True,
            arguments=['-l', plotjuggler_config_file],
            condition=IfCondition(LaunchConfiguration('plotjuggler')),
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=['-d', rviz_config_file],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
