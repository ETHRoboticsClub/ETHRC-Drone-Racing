#!/usr/bin/env python3
"""
Simplified launch file - launches only the MPC controller.
Launch MAVROS separately in another terminal.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch RViz for visualization'
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

    # Set LD_LIBRARY_PATH for ACADOS
    current_ld_path = os.environ.get('LD_LIBRARY_PATH', '')
    new_ld_path = f"{acados_lib_path}:{current_ld_path}" if current_ld_path else acados_lib_path

    set_ld_library_path = SetEnvironmentVariable(
        'LD_LIBRARY_PATH',
        new_ld_path
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
        emulate_tty=True,
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
        use_rviz_arg,

        # Environment variables
        set_acados_source,
        set_ld_library_path,

        # Nodes
        drone_mpc_node,
        rviz_node,
    ])
