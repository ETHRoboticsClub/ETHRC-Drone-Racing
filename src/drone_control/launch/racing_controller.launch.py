from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
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
    waypoints_file = LaunchConfiguration('waypoints_file')
    verbose_input = LaunchConfiguration('verbose_input')
    simulation_mode = LaunchConfiguration('simulation_mode')
    use_mavsdk = LaunchConfiguration('use_mavsdk')
    state_topic = LaunchConfiguration('state_topic')
    state_msg_type = LaunchConfiguration('state_msg_type')
    publish_simulator_action = LaunchConfiguration('publish_simulator_action')
    simulator_action_topic = LaunchConfiguration('simulator_action_topic')
    mpcc_stabilization_sec = LaunchConfiguration('mpcc_stabilization_sec')

    acados_source_dir = LaunchConfiguration('acados_source_dir')
    acados_lib_dir = LaunchConfiguration('acados_lib_dir')

    def as_bool(value):
        return value.lower() in ('true', '1', 'yes', 'y')

    def create_nodes(context, *args, **kwargs):
        controller_param_list = [controller_params]

        # Optional overrides. Leaving these empty lets the YAML config
        # continue supplying its default values.
        if context.launch_configurations.get('controller_type', ''):
            controller_param_list.append({
                'controller_type':
                    context.launch_configurations['controller_type']
            })

        if context.launch_configurations.get('waypoints_file', ''):
            controller_param_list.append({
                'waypoints_file':
                    context.launch_configurations['waypoints_file']
            })

        # Boolean parameters
        controller_param_list.append({
            'verbose_input':
                as_bool(context.launch_configurations['verbose_input']),
            'simulation_mode':
                as_bool(context.launch_configurations['simulation_mode']),
            'use_mavsdk':
                as_bool(context.launch_configurations['use_mavsdk']),
            'publish_simulator_action':
                as_bool(
                    context.launch_configurations[
                        'publish_simulator_action'
                    ]
                ),
            'print_mpc_inputs':
                as_bool(
                    context.launch_configurations[
                        'print_mpc_inputs'
                    ]
                ),
        })

        # String parameters
        controller_param_list.append({
            'state_topic':
                context.launch_configurations['state_topic'],
            'state_msg_type':
                context.launch_configurations['state_msg_type'],
            'simulator_action_topic':
                context.launch_configurations['simulator_action_topic'],
        })

        # Numeric parameters
        controller_param_list.append({
            'mpcc_stabilization_sec':
                float(
                    context.launch_configurations[
                        'mpcc_stabilization_sec'
                    ]
                ),
            'mpc_input_debug_interval_sec':
                float(
                    context.launch_configurations[
                        'mpc_input_debug_interval_sec'
                    ]
                ),
        })

        controller_node = Node(
            package="drone_control",
            executable="racing_controller_node",
            name="drone_controller_node",
            parameters=controller_param_list,
            emulate_tty=True,
            output="screen",
        )

        actions = [controller_node]

        # Preserve HEAD's "use_mavsdk" behavior while also preserving
        # feature/mpc_deploy's 3-second startup delay.
        if as_bool(context.launch_configurations['use_mavsdk']):
            mavsdk_node = Node(
                package="my_mavsdk",
                executable="mavsdk_client",
                name="mavsdk_client",
                parameters=[mavsdk_client_params],
                output="screen",
            )

            actions.append(
                TimerAction(period=3.0, actions=[mavsdk_node])
            )

        return actions

    return LaunchDescription([
        DeclareLaunchArgument('plotjuggler', default_value='true',
            description='Launch PlotJuggler alongside the controller'),
        DeclareLaunchArgument('controller_type', default_value='',
            description='Controller type: mpc, mpcc, mpcc_pp, mpc_rotor, position_ned, rl; leave empty to use config file default'),
        DeclareLaunchArgument('waypoints_file', default_value='',
            description='Path to YAML file with waypoints for the controller; leave empty to disable default trajectory'),
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
        DeclareLaunchArgument('print_mpc_inputs', default_value='false',
            description='Print the current state and goal passed into MPC'),
        DeclareLaunchArgument('mpc_input_debug_interval_sec', default_value='1.0',
            description='Minimum seconds between MPC input log lines'),
        DeclareLaunchArgument('acados_source_dir', default_value='/home/aero/acados',
            description='ACADOS source directory used by MPC code generation'),
        DeclareLaunchArgument('acados_lib_dir', default_value='/home/aero/acados/lib',
            description='Directory containing ACADOS shared libraries'),

        SetEnvironmentVariable('ACADOS_SOURCE_DIR', acados_source_dir),
        SetEnvironmentVariable(
            'LD_LIBRARY_PATH',
            [EnvironmentVariable('LD_LIBRARY_PATH', default_value=''), ':', acados_lib_dir],
        ),

        OpaqueFunction(function=create_nodes),

        Node(
            package="plotjuggler",
            executable="plotjuggler",
            output="screen",
            emulate_tty=True,
            arguments=['-l', plotjuggler_config_file],
            condition=IfCondition(LaunchConfiguration('plotjuggler')),
        ),

    ])
