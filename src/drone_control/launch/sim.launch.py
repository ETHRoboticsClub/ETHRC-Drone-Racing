from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# ── Visualization toggles ──────────────────────────────────────────────────────
ENABLE_RVIZ        = True    # RViz2 waypoint/trajectory visualizer
ENABLE_PLOTJUGGLER = False   # PlotJuggler real-time signal plotter
VERBOSE_INPUT      = False   # Print verbose input information
# ──────────────────────────────────────────────────────────────────────────────


def generate_launch_description():

    pkg = FindPackageShare('drone_control')
    controller_launch = PythonLaunchDescriptionSource(
        PathJoinSubstitution([pkg, 'launch', 'drone_controller.launch.py']))

    own_simulator_controller = IncludeLaunchDescription(
        controller_launch,
        launch_arguments={
            'controller_type': LaunchConfiguration('controller_type'),
            'waypoints_file':  LaunchConfiguration('waypoints_file'),
            'plotjuggler':     LaunchConfiguration('plotjuggler'),
            'verbose_input':   LaunchConfiguration('verbose_input'),
            'simulation_mode': 'true',
            'use_mavsdk': 'false',
            'state_topic': '/mpcc_pp/sim_state',
            'state_msg_type': 'Float64MultiArray',
            'publish_simulator_action': 'true',
            'simulator_action_topic': '/mpcc_pp/mpc_action',
            'mpcc_stabilization_sec': '0.0',
        }.items(),
        condition=IfCondition(LaunchConfiguration('own_simulator')),
    )

    px4_controller = IncludeLaunchDescription(
        controller_launch,
        launch_arguments={
            'controller_type': LaunchConfiguration('controller_type'),
            'waypoints_file':  LaunchConfiguration('waypoints_file'),
            'plotjuggler':     LaunchConfiguration('plotjuggler'),
            'verbose_input':   LaunchConfiguration('verbose_input'),
            'simulation_mode': 'true',
            'use_mavsdk': LaunchConfiguration('use_mavsdk'),
            'state_topic': 'drone_state_sim',
            'state_msg_type': 'Float64MultiArray',
            'publish_simulator_action': 'false',
            'simulator_action_topic': '/mpcc_pp/mpc_action',
            'mpcc_stabilization_sec': LaunchConfiguration('mpcc_stabilization_sec'),
        }.items(),
        condition=UnlessCondition(LaunchConfiguration('own_simulator')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('controller_type', default_value='mpcc_pp',
            description='Controller type: mpc, mpcc, mpcc_pp, mpc_rotor, position_ned, rl'),
        DeclareLaunchArgument('waypoints_file',
            default_value=PathJoinSubstitution([pkg, 'config', 'gates_config.yaml']),
            description='Path to YAML file with gates (MPCC) or waypoints (MPC/other)'),
        DeclareLaunchArgument('plotjuggler', default_value='true' if ENABLE_PLOTJUGGLER else 'false',
            description='Launch PlotJuggler alongside the controller'),
        DeclareLaunchArgument('own_simulator', default_value='true',
            description='Use the lightweight MPCC++ dynamics simulator instead of PX4/Gazebo'),
        DeclareLaunchArgument('use_mavsdk', default_value='false',
            description='Start MavSDK client for the PX4/Gazebo pipeline'),
        DeclareLaunchArgument('mpcc_stabilization_sec', default_value='3.0',
            description='Legacy PX4/Gazebo MPC hover time before MPCC racing'),
        DeclareLaunchArgument('rviz', default_value='true' if ENABLE_RVIZ else 'false',
            description='Launch RViz2 alongside the visualizer'),
        DeclareLaunchArgument('verbose_input',
            default_value='true' if VERBOSE_INPUT else 'false',
            description='Print thrust and body-rate debug info from MPCC and the publish step'),
        DeclareLaunchArgument('simulator_debug', default_value='false',
            description='Print current state, received command, and next state in the simulator'),
        DeclareLaunchArgument('simulator_debug_every', default_value='1',
            description='Print one simulator dynamics debug line every N received actions'),

        Node(
            package='mpcc_pp',
            executable='mpcc_pp_dynamics_simulator',
            name='mpcc_pp_dynamics_simulator',
            output='screen',
            parameters=[{
                'action_input_topic': '/mpcc_pp/mpc_action',
                'external_state_output_topic': 'drone_state_sim',
                'pose_output_topic': '/mpcc_pp/sim_pose',
                'dt': 0.01,
                'periodic_publish': True,
                'hold_initial_until_first_action': True,
                'integration_steps': 0,
                'publish_external_state': True,
                'debug_dynamics': LaunchConfiguration('simulator_debug'),
                'debug_print_every': LaunchConfiguration('simulator_debug_every'),
            }],
            condition=IfCondition(LaunchConfiguration('own_simulator')),
        ),

        own_simulator_controller,
        px4_controller,

        TimerAction(period=2.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([pkg, 'launch', 'visualizer.launch.py'])),
                launch_arguments={
                    'waypoints_file': LaunchConfiguration('waypoints_file'),
                    'rviz':           LaunchConfiguration('rviz'),
                }.items(),
            ),
        ]),
    ])
