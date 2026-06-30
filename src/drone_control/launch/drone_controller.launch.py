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
    acados_source_dir = LaunchConfiguration('acados_source_dir')
    acados_lib_dir = LaunchConfiguration('acados_lib_dir')

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
        if context.launch_configurations.get('print_mpc_inputs', ''):
            print_inputs_val = context.launch_configurations['print_mpc_inputs'].lower()
            controller_param_list.append(
                {'print_mpc_inputs': print_inputs_val in ('true', '1', 'yes', 'y')}
            )
        if context.launch_configurations.get('mpc_input_debug_interval_sec', ''):
            controller_param_list.append({
                'mpc_input_debug_interval_sec':
                    float(context.launch_configurations['mpc_input_debug_interval_sec'])
            })

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

        return [
            controller_node,
            TimerAction(period=3.0, actions=[mavsdk_node]),
        ]

    return LaunchDescription([
        DeclareLaunchArgument('plotjuggler', default_value='true',
            description='Launch PlotJuggler alongside the controller'),
        DeclareLaunchArgument('controller_type', default_value='',
            description='Controller type: mpc, mpcc, mpcc_pp, mpc_rotor, position_ned, rl; leave empty to use config file default'),
        DeclareLaunchArgument('waypoints_file', default_value='',
            description='Path to YAML file with waypoints for the controller; leave empty to disable default trajectory'),
        DeclareLaunchArgument('verbose_input', default_value='false',
            description='Print thrust and body-rate debug info'),
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
