from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# ── Visualization toggles ──────────────────────────────────────────────────────
ENABLE_RVIZ        = True   # RViz2 waypoint/trajectory visualizer
ENABLE_PLOTJUGGLER = False   # PlotJuggler real-time signal plotter
VERBOSE_INPUT    = True     # Print verbose input information
# ──────────────────────────────────────────────────────────────────────────────


def generate_launch_description():

    pkg = FindPackageShare('drone_control')

    return LaunchDescription([
        DeclareLaunchArgument('controller_type', default_value='mpcc',
            description='Controller type: mpc, mpcc, mpc_rotor, position_ned, rl'),
        DeclareLaunchArgument('waypoints_file',
            default_value=PathJoinSubstitution([pkg, 'config', 'gates_config.yaml']),
            description='Path to YAML file with gates (MPCC) or waypoints (MPC/other)'),
        DeclareLaunchArgument('plotjuggler', default_value='true' if ENABLE_PLOTJUGGLER else 'false',
            description='Launch PlotJuggler alongside the controller'),
        DeclareLaunchArgument('rviz', default_value='true' if ENABLE_RVIZ else 'false',
            description='Launch RViz2 alongside the visualizer'),

        DeclareLaunchArgument('verbose_input',
            default_value='true' if VERBOSE_INPUT else 'false',
            description='Print thrust and body-rate debug info from MPCC and the publish step'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg, 'launch', 'drone_controller.launch.py'])),
            launch_arguments={
                'controller_type': LaunchConfiguration('controller_type'),
                'waypoints_file':  LaunchConfiguration('waypoints_file'),
                'plotjuggler':     LaunchConfiguration('plotjuggler'),
                'verbose_input':   LaunchConfiguration('verbose_input'),
            }.items(),
        ),

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
