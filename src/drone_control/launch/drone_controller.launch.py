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
    simulation_mode = LaunchConfiguration('simulation_mode')
    use_mavsdk = LaunchConfiguration('use_mavsdk')
    state_topic = LaunchConfiguration('state_topic')
    state_msg_type = LaunchConfiguration('state_msg_type')
    publish_simulator_action = LaunchConfiguration('publish_simulator_action')
    simulator_action_topic = LaunchConfiguration('simulator_action_topic')
    mpcc_stabilization_sec = LaunchConfiguration('mpcc_stabilization_sec')

    controller_node = Node(
        package="drone_control",
        executable="drone_controller_node",
        name="drone_controller_node",
        parameters=[controller_params, {
            'controller_type': controller_type,
            'waypoints_file': waypoints_file,
            'verbose_input': verbose_input,
            'simulation_mode': simulation_mode,
            'use_mavsdk': use_mavsdk,
            'state_topic': state_topic,
            'state_msg_type': state_msg_type,
            'publish_simulator_action': publish_simulator_action,
            'simulator_action_topic': simulator_action_topic,
            'mpcc_stabilization_sec': mpcc_stabilization_sec,
        }],
        emulate_tty=True,
        output="screen",
    )

    mavsdk_node = Node(
        package="my_mavsdk",
        executable="mavsdk_client",
        name="mavsdk_client",
        parameters=[mavsdk_client_params],
        output="screen",
        condition=IfCondition(use_mavsdk),
    )

    return LaunchDescription([
        DeclareLaunchArgument('plotjuggler', default_value='true',
            description='Launch PlotJuggler alongside the controller'),
        DeclareLaunchArgument('controller_type', default_value='mpcc_pp',
            description='Controller type: mpc, mpcc, mpcc_pp, mpc_rotor, position_ned, rl'),
        DeclareLaunchArgument('waypoints_file', default_value=default_waypoints_file,
            description='Path to YAML file with waypoints for the controller'),
        DeclareLaunchArgument('verbose_input', default_value='false',
            description='Print thrust and body-rate debug info'),
        DeclareLaunchArgument('simulation_mode', default_value='false',
            description='Enable simulation-only Optuna progress/crash/missed-gate metrics'),
        DeclareLaunchArgument('use_mavsdk', default_value='true',
            description='Start MavSDK client and arm PX4/offboard'),
        DeclareLaunchArgument('state_topic', default_value='drone_state_sim',
            description='Drone state topic consumed by the controller'),
        DeclareLaunchArgument('state_msg_type', default_value='Float64MultiArray',
            description='State message type: PoseStamped or Float64MultiArray'),
        DeclareLaunchArgument('publish_simulator_action', default_value='false',
            description='Publish raw MPCC++ action for the lightweight simulator'),
        DeclareLaunchArgument('simulator_action_topic', default_value='/mpcc_pp/mpc_action',
            description='Topic for raw MPCC++ simulator actions'),
        DeclareLaunchArgument('mpcc_stabilization_sec', default_value='3.0',
            description='MPC hover time before MPCC racing; use 0 for ROS-only simulator'),

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
