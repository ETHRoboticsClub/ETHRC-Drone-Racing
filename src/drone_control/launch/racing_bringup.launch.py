import os
import socket
import launch.conditions

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # --- Machine Detection ---
    hostname = socket.gethostname()
    is_jetson = hostname in ['zephyrus', 'boreas', 'jetson']
    
    # --- Packages ---
    pkg_drone_control = get_package_share_directory('drone_control')
    pkg_msp_control = get_package_share_directory('betaflight_msp_controller')
    pkg_blackbox = get_package_share_directory('betaflight_blackbox_driver')
    pkg_vicon = get_package_share_directory('vicon_receiver')

    # --- Arguments ---
    default_imu_port = '/dev/ttyTHS1' if is_jetson else '/dev/ttyUSB0'
    default_msp_port = '/dev/ttyTCU0' if is_jetson else '/dev/ttyACM0'

    arg_imu_port = DeclareLaunchArgument('imu_port', default_value=default_imu_port,
                                            description='Dedicated UART for high-rate Blackbox IMU telemetry')
    arg_msp_port = DeclareLaunchArgument('msp_port', default_value=default_msp_port,
                                            description='Dedicated UART/USB-VCP for MSP Control commands')
    arg_attitude_topic = DeclareLaunchArgument('attitude_cmd_topic', default_value='/drone_attitude_cmd')
    arg_use_vicon = DeclareLaunchArgument('use_vicon', default_value='true')
    arg_use_rc_sim = DeclareLaunchArgument('use_rc_sim', default_value='true',
                                             description='Whether to use MSP RC simulation for arming')

    # --- 1. IMU DRIVER: Betaflight Blackbox ---
    # This C++ node handles the high-rate IMU telemetry stream (read-only)
    blackbox_node = Node(
        package='betaflight_blackbox_driver',
        executable='blackbox_driver_node',
        name='imu_driver_node',
        parameters=[{
            'port': LaunchConfiguration('imu_port'),
            'baud': 1500000,
            'imu_topic': '/imu0',
            'enable_msp_control': False  # Explicitly disabled in separate port mode
        }],
        output='screen'
    )

    # --- 2. MSP Bridge: RC Override (Arming/Control) ---
    # This Python node handles the dedicated port to simulate the RC receiver
    msp_bridge_node = Node(
        package='betaflight_msp_controller',
        executable='msp_bridge',
        name='msp_bridge',
        parameters=[{
            'port': LaunchConfiguration('msp_port'),
            'baudrate': 115200 if not is_jetson else 1500000,
            'attitude_cmd_topic': LaunchConfiguration('attitude_cmd_topic')
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


    return LaunchDescription([
        arg_imu_port,
        arg_msp_port,
        arg_attitude_topic,
        arg_use_vicon,
        arg_use_rc_sim,
        blackbox_node,
        msp_bridge_node,
        vicon_launch,
        control_launch,
    ])
