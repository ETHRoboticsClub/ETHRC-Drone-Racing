#!/usr/bin/env python3
"""
Launch file for Drone MPC Controller using MAVSDK.

This is much simpler than the MAVROS version because MAVSDK
handles the connection internally - no separate MAVROS node needed!

Usage:
    ros2 launch drone_mpc_control drone_mpc_mavsdk.launch.py

Arguments:
    mavsdk_port: Port for MAVSDK connection (default: 14540 for PX4 SITL)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for MAVSDK-based MPC controller."""

    # ========================================================================
    # LAUNCH ARGUMENTS
    # ========================================================================

    mavsdk_port_arg = DeclareLaunchArgument(
        'mavsdk_port',
        default_value='14540',
        description='Port for MAVSDK connection to PX4 (14540 for SITL)'
    )

    # ========================================================================
    # PACKAGE DIRECTORIES
    # ========================================================================

    drone_mpc_control_dir = get_package_share_directory('drone_mpc_control')

    # ========================================================================
    # ENVIRONMENT VARIABLES
    # ========================================================================

    # ACADOS paths (required for MPC solver)
    acados_source_dir = os.path.expanduser('~/acados')
    acados_lib_path = os.path.join(acados_source_dir, 'lib')

    set_acados_source = SetEnvironmentVariable(
        'ACADOS_SOURCE_DIR',
        acados_source_dir
    )

    # Add ACADOS library to LD_LIBRARY_PATH
    current_ld_path = os.environ.get('LD_LIBRARY_PATH', '')
    new_ld_path = f"{acados_lib_path}:{current_ld_path}" if current_ld_path else acados_lib_path

    set_ld_library_path = SetEnvironmentVariable(
        'LD_LIBRARY_PATH',
        new_ld_path
    )

    # ========================================================================
    # PARAMETER FILES
    # ========================================================================

    # Load MPC and drone parameters from YAML files
    mpc_params_file = os.path.join(drone_mpc_control_dir, 'config', 'mpc_params.yaml')
    drone_params_file = os.path.join(drone_mpc_control_dir, 'config', 'drone_params.yaml')

    # ========================================================================
    # NODES
    # ========================================================================

    # MPC Controller Node (with MAVSDK)
    drone_mpc_node = Node(
        package='drone_mpc_control',
        executable='drone_mpc_mavsdk',  # New MAVSDK-based executable
        name='drone_mpc_controller',
        output='screen',
        parameters=[
            mpc_params_file,
            drone_params_file,
            {'mavsdk_port': LaunchConfiguration('mavsdk_port')}
        ],
        emulate_tty=True,  # For colored output
    )

    # ========================================================================
    # LAUNCH DESCRIPTION
    # ========================================================================

    return LaunchDescription([
        # Arguments
        mavsdk_port_arg,

        # Environment
        set_acados_source,
        set_ld_library_path,

        # Nodes
        drone_mpc_node,
    ])
