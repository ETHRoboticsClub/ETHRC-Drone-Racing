import os
import socket
import launch.conditions

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, LogInfo

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
    arg_view_stream = DeclareLaunchArgument('view_stream', default_value='true',
                                              description='Whether to start the video streaming/republishing pipeline')
    arg_video_bitrate = DeclareLaunchArgument('video_bitrate', default_value='4000000')
    arg_video_width = DeclareLaunchArgument('video_width', default_value='1920')
    arg_video_height = DeclareLaunchArgument('video_height', default_value='1080')
    arg_video_fps = DeclareLaunchArgument('video_fps', default_value='30')
    arg_video_flip = DeclareLaunchArgument('video_flip', default_value='2')
    
    arg_msp_bench_mode = DeclareLaunchArgument('msp_bench_mode', default_value='false',
                                               description='Disable MSP Bridge to allow standalone TX emulator')
    arg_load_bf_config = DeclareLaunchArgument('load_bf_config', default_value='false',
                                               description='Automatically load Betaflight config on startup')
    
    # Paths to config files
    ws_root = os.path.abspath(os.path.join(pkg_drone_control, '../../../../'))
    bench_config_path = os.path.join(ws_root, 'docs', 'docs', 'files', 'betaflight_conf', 'ZEPHYRUS_MSP_BENCH.txt')
    flight_config_path = os.path.join(ws_root, 'docs', 'docs', 'files', 'betaflight_conf', 'zephyrus_KAKUTEH7.txt')

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
        condition=launch.conditions.IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('use_rc_sim'), "' == 'true' and '",
                LaunchConfiguration('msp_bench_mode'), "' == 'false'"
            ])
        ),
        output='screen'
    )
    
    # --- 2.1 Bench Mode Warning ---
    bench_mode_warning = LogInfo(
        msg="WARNING: msp_bench_mode is active! The msp_bridge node is disabled. "
            "Please run 'ros2 run betaflight_msp_controller msp_tx_emulator' in another terminal.",
        condition=launch.conditions.IfCondition(LaunchConfiguration('msp_bench_mode'))
    )
    
    # --- 2.2 Auto-load Betaflight Configuration ---
    # Select config file based on bench mode flag
    selected_config_path = PythonExpression([
        "'", bench_config_path, "' if '", LaunchConfiguration('msp_bench_mode'), "' == 'true' else '", flight_config_path, "'"
    ])
    
    load_config_process = ExecuteProcess(
        cmd=['ros2', 'run', 'betaflight_msp_controller', 'load_bf_config', 
             selected_config_path, '--port', LaunchConfiguration('msp_port')],
        output='screen',
        condition=launch.conditions.IfCondition(LaunchConfiguration('load_bf_config'))
    )

    # --- 3. VIDEO: H.264 Publisher (Jetson) or Republisher (GS) ---
    # Publisher: Runs only on Jetson to capture and compress CSI camera frames
    video_publisher_node = Node(
        package='jetson_h264_publisher',
        executable='encoder_node',
        name='jetson_h264_publisher',
        parameters=[{
            'bitrate': LaunchConfiguration('video_bitrate'),
            'width': LaunchConfiguration('video_width'),
            'height': LaunchConfiguration('video_height'),
            'framerate': LaunchConfiguration('video_fps'),
            'flip_method': LaunchConfiguration('video_flip')
        }],
        condition=launch.conditions.IfCondition('true' if is_jetson else 'false'),
        output='screen'
    )

    # Republisher: Runs on GS to decode the H.264 stream back to raw images for RQT/Foxglove
    video_republisher_node = Node(
        package='image_transport',
        executable='republish',
        name='h264_republisher',
        arguments=['h264'],
        remappings=[
            ('in/h264', '/image_raw/h264'),
            ('out', '/image_raw')
        ],
        parameters=[{
            'qos_overrides./image_raw/h264.subscription.reliability': 'best_effort'
        }],
        condition=launch.conditions.IfCondition(
            launch.substitutions.PythonExpression([
                "'", LaunchConfiguration('view_stream'), "' == 'true' and not ", str(is_jetson)
            ])
        ),
        output='screen'
    )

    # --- 4. VICON: Ground Truth & Timestamping ---
    vicon_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_vicon, 'launch', 'client.launch.py')
        ),
        launch_arguments={'hostname': '10.200.117.1'}.items() # Standard lab Vicon IP
    )

    # --- 5. CONTROL: Drone Controller (MPC) ---
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
        arg_view_stream,
        arg_video_bitrate,
        arg_video_width,
        arg_video_height,
        arg_video_fps,
        arg_video_flip,
        arg_msp_bench_mode,
        arg_load_bf_config,
        blackbox_node,
        msp_bridge_node,
        bench_mode_warning,
        load_config_process,
        video_publisher_node,
        video_republisher_node,
        vicon_launch,
        control_launch,
    ])
