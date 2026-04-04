import os
import launch.conditions

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # --- Packages ---
    pkg_drone_control = get_package_share_directory('drone_control')
    pkg_msp_control = get_package_share_directory('betaflight_msp_controller')
    pkg_blackbox = get_package_share_directory('betaflight_blackbox_driver')
    pkg_vicon = get_package_share_directory('vicon_receiver')
    pkg_perception = get_package_share_directory('aruco_pose_estimation')

    # --- Arguments ---
    arg_shared_port = DeclareLaunchArgument('shared_port', default_value='/dev/ttyTHS1',
                                            description='Shared USB/UART for MSP Commands and Blackbox IMU')
    arg_rc_sim_port = DeclareLaunchArgument('rc_sim_port', default_value='/dev/ttyACM0',
                                            description='USB Port for MSP RC Simulation (Arming)')
    arg_use_vicon = DeclareLaunchArgument('use_vicon', default_value='true')
    arg_use_rc_sim = DeclareLaunchArgument('use_rc_sim', default_value='true',
                                             description='Whether to use MSP RC simulation for arming')

    # --- 1. SHARED UART: Blackbox + MSP Control ---
    # This C++ node handles the high-rate IMU and attitude command injection
    blackbox_node = Node(
        package='betaflight_blackbox_driver',
        executable='blackbox_driver_node',
        name='shared_uart_node',
        parameters=[{
            'port': LaunchConfiguration('shared_port'),
            'baud': 1500000,
            'imu_topic': '/imu0',
            'attitude_command_topic': '/drone_attitude_cmd'
        }],
        output='screen'
    )

    # --- 2. MSP Bridge: RC Override (Arming/Control) ---
    # This Python node handles the VCP port to simulate the RC receiver
    msp_bridge_node = Node(
        package='betaflight_msp_controller',
        executable='msp_bridge',
        name='msp_bridge',
        parameters=[{
            'port': LaunchConfiguration('rc_sim_port'),
            'baudrate': 115200
        }],
        condition=launch.conditions.IfCondition(LaunchConfiguration('use_rc_sim')),
        output='screen'
    )

    # --- 3. VICON: Ground Truth & Timestamping ---
    vicon_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_vicon, 'launch', 'client.launch.py')
        ),
        launch_arguments={'hostname': '10.200.117.1'}.items() # Standard lab Vicon IP
    )

    # --- 4. CONTROL: Drone Controller (MPC) ---
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_drone_control, 'launch', 'drone_controller.launch.py')
        ),
        launch_arguments={
            'plotjuggler': 'false',
            'use_mavsdk': 'false'
        }.items() 
    )

    # --- 5. PERCEPTION: ArUco Estimation ---
    # Set publish_rgb:=true if you want to see the video stream
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_perception, 'launch', 'aruco_pose_estimation.launch.py')
        ),
        launch_arguments={
            'rviz': 'false',
            'publish_debug_image': 'true'
        }.items()
    )

    return LaunchDescription([
        arg_shared_port,
        arg_rc_sim_port,
        arg_use_vicon,
        arg_use_rc_sim,
        blackbox_node,
        msp_bridge_node,
        vicon_launch,
        control_launch,
        perception_launch
    ])
