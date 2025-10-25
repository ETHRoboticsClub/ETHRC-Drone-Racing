#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='true',
        description='Use simulation (true/false)'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch RViz for visualization'
    )

    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='udp://:14540@127.0.0.1:14580',
        description='FCU connection URL (default for PX4 SITL)'
    )

    # Get package directories
    drone_mpc_control_dir = get_package_share_directory('drone_mpc_control')

    # Environment variables for ACADOS
    acados_source_dir = os.path.expanduser('~/acados')
    acados_lib_path = os.path.join(acados_source_dir, 'lib')

    set_acados_source = SetEnvironmentVariable(
        'ACADOS_SOURCE_DIR',
        acados_source_dir
    )

    # Note: LD_LIBRARY_PATH should be set in .bashrc for persistence
    # We set it here as well for the launch context
    current_ld_path = os.environ.get('LD_LIBRARY_PATH', '')
    new_ld_path = f"{acados_lib_path}:{current_ld_path}" if current_ld_path else acados_lib_path

    set_ld_library_path = SetEnvironmentVariable(
        'LD_LIBRARY_PATH',
        new_ld_path
    )

    # MAVROS Node
    # Launch MAVROS directly as a node
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        output='screen',
        parameters=[{
            'fcu_url': LaunchConfiguration('fcu_url'),
            'gcs_url': '',
            'target_system_id': 1,
            'target_component_id': 1,
            'fcu_protocol': 'v2.0',
        }],
        emulate_tty=True,
    )

    # Load parameter files
    mpc_params_file = os.path.join(drone_mpc_control_dir, 'config', 'mpc_params.yaml')
    drone_params_file = os.path.join(drone_mpc_control_dir, 'config', 'drone_params.yaml')

    # MPC Controller Node
    drone_mpc_node = Node(
        package='drone_mpc_control',
        executable='drone_mpc_node',
        name='drone_mpc_controller',
        output='screen',
        parameters=[
            mpc_params_file,
            drone_params_file
        ],
        emulate_tty=True,  # For proper colored output
    )

    # RViz Node (optional)
    rviz_config_file = os.path.join(drone_mpc_control_dir, 'rviz', 'drone_view.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    return LaunchDescription([
        # Arguments
        sim_arg,
        use_rviz_arg,
        fcu_url_arg,

        # Environment variables
        set_acados_source,
        set_ld_library_path,

        # Nodes
        mavros_node,
        drone_mpc_node,
        rviz_node,
    ])
