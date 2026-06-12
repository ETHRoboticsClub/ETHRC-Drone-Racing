import os
import socket
import launch.conditions

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # --- Machine Detection ---
    hostname = socket.gethostname()
    is_jetson = hostname in ['zephyrus', 'boreas', 'jetson']
    
    # --- Packages ---
    pkg_drone_control = get_package_share_directory('drone_control')
    ws_root = os.path.abspath(os.path.join(pkg_drone_control, '../../../../'))

    # --- Paths for SITL ---
    os.environ['GZ_VERSION'] = 'harmonic'
    models_dir = os.path.join(ws_root, 'sim', 'gazebo', 'models')
    gz_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    if models_dir not in gz_resource_path:
        os.environ['GZ_SIM_RESOURCE_PATH'] = f"{models_dir}:{gz_resource_path}" if gz_resource_path else models_dir

    plugins_dir = os.path.join(ws_root, 'ext', 'aeroloop_gazebo', 'plugins', 'build')
    gz_plugin_path = os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
    if plugins_dir not in gz_plugin_path:
        os.environ['GZ_SIM_SYSTEM_PLUGIN_PATH'] = f"{plugins_dir}:{gz_plugin_path}" if gz_plugin_path else plugins_dir

    # --- Arguments ---
    arg_sitl = DeclareLaunchArgument('sitl', default_value='false',
                                     description='Whether to run in SITL mode with Gazebo and Betaflight')

    # Try to find NVIDIA Isaac ROS encoder/decoder package paths to avoid crashes on non-Jetson hosts
    has_encoder = False
    try:
        get_package_share_directory('isaac_ros_h264_encoder')
        has_encoder = True
    except PackageNotFoundError:
        pass

    has_decoder = False
    try:
        get_package_share_directory('isaac_ros_h264_decoder')
        has_decoder = True
    except PackageNotFoundError:
        pass

    default_imu_port = '/dev/ttyTHS1' if is_jetson else '/dev/ttyUSB0'
    default_msp_port = '/dev/ttyTCU0' if is_jetson else '/dev/ttyACM0'

    arg_imu_port = DeclareLaunchArgument('imu_port', default_value=default_imu_port,
                                            description='Dedicated UART for high-rate Blackbox IMU telemetry')
    arg_imu_source = DeclareLaunchArgument('imu_source', default_value='blackbox',
                                            choices=['blackbox', 'msp'],
                                            description='Source of IMU telemetry: "blackbox" or "msp"')
    arg_imu_baud = DeclareLaunchArgument('imu_baud', default_value='1500000',
                                          description='Baudrate for the IMU telemetry serial port')
    arg_msp_port = DeclareLaunchArgument('msp_port', default_value=default_msp_port,
                                            description='Dedicated UART/USB-VCP for MSP Control commands')
    arg_attitude_topic = DeclareLaunchArgument('attitude_cmd_topic', default_value='/drone_attitude_cmd')
    arg_target_hz = DeclareLaunchArgument('target_hz', default_value='150',
                                          description='Target frequency for the published IMU messages')
    arg_use_vicon = DeclareLaunchArgument('use_vicon', default_value='true')
    arg_vicon_subject = DeclareLaunchArgument('vicon_subject', default_value=hostname)
    arg_vicon_segment = DeclareLaunchArgument('vicon_segment', default_value=hostname)
    arg_use_rc_sim = DeclareLaunchArgument('use_rc_sim', default_value='true',
                                             description='Whether to use MSP RC simulation for arming')
    arg_view_stream = DeclareLaunchArgument('view_stream', default_value='true',
                                               description='Start the secondary RTP/H.264 video path')
    
    # Video defaults: 720p @ 60fps is optimized for racing
    arg_video_bitrate = DeclareLaunchArgument('video_bitrate', default_value='4000000')
    arg_video_width = DeclareLaunchArgument('video_width', default_value='1280')
    arg_video_height = DeclareLaunchArgument('video_height', default_value='720')
    arg_video_fps = DeclareLaunchArgument('video_fps', default_value='60')
    arg_video_port = DeclareLaunchArgument('video_port', default_value='5000')
    arg_h264_backend = DeclareLaunchArgument(
        'h264_backend', default_value='isaac_ros',
        description='H.264 backend when view_stream=true: gstreamer_ros, isaac_ros, or gstreamer'
    )
    arg_isaac_h264_qp = DeclareLaunchArgument(
        'isaac_h264_qp', default_value='18',
        description='Isaac ROS H.264 quantization parameter; lower is higher quality/larger output'
    )
    arg_isaac_h264_iframe_interval = DeclareLaunchArgument(
        'isaac_h264_iframe_interval', default_value='30',
        description='Isaac ROS H.264 I-frame interval in frames'
    )
    arg_isaac_h264_profile = DeclareLaunchArgument(
        'isaac_h264_profile', default_value='1',
        description='Isaac ROS H.264 profile passed to the encoder launch'
    )
    arg_isaac_h264_hw_preset = DeclareLaunchArgument(
        'isaac_h264_hw_preset', default_value='0',
        description='Isaac ROS H.264 hardware preset passed to the encoder launch'
    )
    arg_decode_h264 = DeclareLaunchArgument(
        'decode_h264', default_value='false',
        description='Decode /cam0/image_h264 to /image_decoded for local visualization/debugging'
    )
    arg_ground_station_ip = DeclareLaunchArgument(
        'ground_station_ip', default_value='192.168.0.78',
        description='Ground station IPv4 address for the Jetson RTP/H.264 UDP stream'
    )
    arg_publish_raw = DeclareLaunchArgument(
        'publish_raw', default_value='false',
        description='Publish /cam0/image_raw as mono8 for VIO'
    )
    arg_publish_rgb = DeclareLaunchArgument(
        'publish_rgb', default_value='true',
        description='Publish /cam0/image_rgb as the default color camera topic'
    )
    arg_camera_info_url = DeclareLaunchArgument(
        'camera_info_url', default_value='file:///home/aero/proj/ETHRC-Drone-Racing/src/perception/camera_calibration/camera_info.yaml',
        description='Path to standard camera calibration YAML file'
    )
    arg_use_gate_detection = DeclareLaunchArgument(
        'use_gate_detection', default_value='false',
        description='Gate detector is WIP; enable only after gate_detection is built and installed'
    )
    arg_gate_camera_topic = DeclareLaunchArgument(
        'gate_camera_topic', default_value='/cam0/image_rgb',
        description='Gate detector image topic. /cam0/image_rgb has real color; /cam0/image_raw is mono.'
    )
    arg_use_vio = DeclareLaunchArgument('use_vio', default_value='false')
    arg_use_ekf = DeclareLaunchArgument('use_ekf', default_value='false')

    # Bench Mode Arguments from Main
    arg_msp_bench_mode = DeclareLaunchArgument('msp_bench_mode', default_value='false',
                                               description='Disable MSP Bridge to allow standalone TX emulator')
    arg_load_bf_config = DeclareLaunchArgument('load_bf_config', default_value='false',
                                               description='Automatically load Betaflight config on startup')
    
    # Paths to config files (Bench Mode)
    ws_root = os.path.abspath(os.path.join(pkg_drone_control, '../../../../'))
    bench_config_path = os.path.join(ws_root, 'docs', 'docs', 'files', 'betaflight_conf', 'ZEPHYRUS_MSP_BENCH.txt')
    flight_config_path = os.path.join(ws_root, 'docs', 'docs', 'files', 'betaflight_conf', 'zephyrus_KAKUTEH7.txt')

    # RL Controller Arguments
    arg_controller_type = DeclareLaunchArgument('controller_type', default_value='rl',
                                              description='Type of controller: "mpc" or "rl"')
    arg_rate_error_scale = DeclareLaunchArgument('rate_error_scale', default_value='300.0',
                                               description='Torque-to-rate scaling for RL controller')

    # --- 1. IMU DRIVER: Betaflight Blackbox or MSP ---
    blackbox_node = Node(
        package='betaflight_blackbox_driver',
        executable='blackbox_driver_node',
        name='imu_driver_node',
        parameters=[{
            'port': LaunchConfiguration('imu_port'),
            'baud': LaunchConfiguration('imu_baud'),
            'imu_topic': '/imu0',
            'target_hz': LaunchConfiguration('target_hz'),
            'enable_msp_control': False, # Explicitly disabled in separate port mode
            'motor_poles': 14,
            'rpm_topic': '/motor_rpm',
            # IMU validation parameters
            'validate_imu_stream': False,
            'reject_zero_imu': True,
            'max_gyro_deg_s': 4000.0,       # Physical max (2x 2000 DPS full-scale range)
            'max_accel_g': 32.0,            # Physical max
            'max_gyro_jump_deg_s': 1000.0,  # Per-frame jump limit
            'max_accel_jump_g': 8.0,        # Per-frame jump limit
            'max_consecutive_rejects': 5,   # Hard resync after N consecutive bad frames
            'outlier_sigma_k': 4.0,         # Sliding-window outlier detection (sigma)
        }],
        condition=launch.conditions.IfCondition(
            PythonExpression(["'", LaunchConfiguration('sitl'), "' == 'false' and '", LaunchConfiguration('imu_source'), "' == 'blackbox'"])
        ),
        output='screen'
    )

    msp_imu_node = Node(
        package='betaflight_blackbox_driver',
        executable='msp_imu_driver_node',
        name='imu_driver_node',
        parameters=[{
            'port': LaunchConfiguration('imu_port'),
            'baud': LaunchConfiguration('imu_baud'),
            'imu_topic': '/imu0',
            'target_hz': LaunchConfiguration('target_hz'),
            'rpm_topic': '/motor_rpm',
        }],
        condition=launch.conditions.IfCondition(
            PythonExpression(["'", LaunchConfiguration('sitl'), "' == 'false' and '", LaunchConfiguration('imu_source'), "' == 'msp'"])
        ),
        output='screen'
    )

    # --- 2. MSP Bridge: RC Override ---
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
                "'", LaunchConfiguration('sitl'), "' == 'false' and '",
                LaunchConfiguration('use_rc_sim'), "' == 'true' and '",
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
    selected_config_path = PythonExpression([
        "'", bench_config_path, "' if '", LaunchConfiguration('msp_bench_mode'), "' == 'true' else '", flight_config_path, "'"
    ])
    
    load_config_process = ExecuteProcess(
        cmd=['ros2', 'run', 'betaflight_msp_controller', 'load_bf_config', 
             selected_config_path, '--port', LaunchConfiguration('msp_port')],
        output='screen',
        condition=launch.conditions.IfCondition(
            PythonExpression(["'", LaunchConfiguration('load_bf_config'), "' == 'true' and '", LaunchConfiguration('sitl'), "' == 'false'"])
        )
    )

    # --- 3. VIDEO: Team-Standard Camera for perception ---
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('drone_control'), 'launch', 'nitros_camera.launch.py']
            )
        ),
        launch_arguments={
            'sensor_id': '0',
            'sensor_mode': '4',
            'image_width': LaunchConfiguration('video_width'),
            'image_height': LaunchConfiguration('video_height'),
            'framerate': LaunchConfiguration('video_fps'),
            'publish_mono': LaunchConfiguration('publish_raw'),
            'publish_rgb': LaunchConfiguration('publish_rgb'),
            'camera_info_url': LaunchConfiguration('camera_info_url'),
        }.items(),
        condition=launch.conditions.IfCondition(
            PythonExpression([
                "('", LaunchConfiguration('publish_raw'), "' == 'true' or '",
                LaunchConfiguration('publish_rgb'), "' == 'true') and ", str(is_jetson)
            ])
        )
    )

    # --- 3.1 Legacy exclusive H.264 RTP Stream (kept for video-only debug) ---
    video_publisher_node = Node(
        package='jetson_h264_publisher',
        executable='encoder_node',
        name='jetson_h264_streamer',
        parameters=[{
            'ground_station_ip': LaunchConfiguration('ground_station_ip'),
            'port': LaunchConfiguration('video_port'),
            'sensor_id': 0,
            'sensor_mode': 4,
            'width': LaunchConfiguration('video_width'),
            'height': LaunchConfiguration('video_height'),
            'framerate': LaunchConfiguration('video_fps'),
            'bitrate': LaunchConfiguration('video_bitrate'),
            'iframe_interval': 30,
            'preset_level': 0,
            'control_rate': 1,
            'insert_sps_pps': True,
            'payload_type': 96,
            'config_interval': 1,
            'sync': False,
            'async': False,
        }],
        condition=launch.conditions.IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('view_stream'), "' == 'true' and '",
                LaunchConfiguration('publish_raw'), "' == 'false' and '",
                LaunchConfiguration('h264_backend'), "' == 'gstreamer' and ", str(is_jetson)
            ])
        ),
        output='screen'
    )

    gstreamer_stream_enabled_for_perception = LogInfo(
        msg="RTP/H.264 stream enabled inside the perception camera pipeline; CSI is captured once and teed to /cam0/image_raw plus UDP.",
        condition=launch.conditions.IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('view_stream'), "' == 'true' and '",
                LaunchConfiguration('publish_raw'), "' == 'true' and '",
                LaunchConfiguration('h264_backend'), "' == 'gstreamer' and ", str(is_jetson)
            ])
        )
    )

    gstreamer_ros_h264_enabled = LogInfo(
        msg="GStreamer ROS H.264 enabled inside the camera pipeline: /cam0/image_h264. /cam0/image_rgb remains a best-effort color topic for gate detection/debug.",
        condition=launch.conditions.IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('view_stream'), "' == 'true' and '",
                LaunchConfiguration('publish_raw'), "' == 'true' and '",
                LaunchConfiguration('h264_backend'), "' == 'gstreamer_ros' and ", str(is_jetson)
            ])
        )
    )

    encoder_actions = [
        SetRemap(src='image_raw', dst='/cam0/image_raw_nitros'),
        SetRemap(src='image_compressed', dst='/cam0/image_h264'),
    ]
    if has_encoder:
        encoder_actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('isaac_ros_h264_encoder'),
                        'launch',
                        'isaac_ros_h264_encoder.launch.py'
                    ])
                ),
                launch_arguments={
                    'input_width': LaunchConfiguration('video_width'),
                    'input_height': LaunchConfiguration('video_height'),
                    'config': 'pframe_cqp',
                }.items(),
            )
        )

    isaac_h264_encoder = GroupAction(
        actions=encoder_actions,
        condition=launch.conditions.IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('view_stream'), "' == 'true' and '",
                LaunchConfiguration('h264_backend'), "' == 'isaac_ros' and ('",
                LaunchConfiguration('publish_raw'), "' == 'true' or '",
                LaunchConfiguration('publish_rgb'), "' == 'true') and ",
                str(is_jetson)
            ])
        )
    )

    isaac_h264_enabled = LogInfo(
        msg="Isaac ROS H.264 encoder enabled: /cam0/image_raw_nitros -> /cam0/image_h264.",
        condition=launch.conditions.IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('view_stream'), "' == 'true' and '",
                LaunchConfiguration('h264_backend'), "' == 'isaac_ros' and ('",
                LaunchConfiguration('publish_raw'), "' == 'true' or '",
                LaunchConfiguration('publish_rgb'), "' == 'true') and ",
                str(is_jetson)
            ])
        )
    )

    video_receiver = ExecuteProcess(
        cmd=[
            'gst-launch-1.0',
            'udpsrc', PythonExpression(["'port=' + '", LaunchConfiguration('video_port'), "'"]),
            'caps=application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96',
            '!', 'rtpjitterbuffer', 'latency=0', 'drop-on-latency=true',
            '!', 'rtph264depay',
            '!', 'avdec_h264',
            '!', 'videoconvert',
            '!', 'autovideosink', 'sync=false', 'async=false',
        ],
        condition=launch.conditions.IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('view_stream'), "' == 'true' and not ", str(is_jetson),
                " and '", LaunchConfiguration('h264_backend'), "' == 'gstreamer'"
            ])
        ),
        output='screen'
    )

    decoder_actions = [
        SetRemap(src='image_compressed', dst='/cam0/image_h264'),
        SetRemap(src='image_uncompressed', dst='/image_decoded'),
    ]
    if has_decoder:
        decoder_actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('isaac_ros_h264_decoder'),
                        'launch',
                        'isaac_ros_h264_decoder.launch.py'
                    ])
                )
            )
        )

    isaac_h264_decoder = GroupAction(
        actions=decoder_actions,
        condition=launch.conditions.IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('view_stream'), "' == 'true' and '",
                LaunchConfiguration('h264_backend'), "' in ['isaac_ros', 'gstreamer_ros'] and (not ", str(is_jetson),
                " or '", LaunchConfiguration('decode_h264'), "' == 'true')"
            ])
        )
    )

    gate_detection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('gate_detection'), 'launch', 'gate_detection.launch.py']
            )
        ),
        launch_arguments={
            'camera_topic': LaunchConfiguration('gate_camera_topic'),
            'output_topic': '/gate_detection/drone_pose',
        }.items(),
        condition=launch.conditions.IfCondition(LaunchConfiguration('use_gate_detection'))
    )

    ov_config_path = PathJoinSubstitution(
        [FindPackageShare('ov_msckf'), 'config', 'zephyrus_real', 'estimator_config.yaml']
    )
    vio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('ov_msckf'), 'launch', 'subscribe.launch.py']
            )
        ),
        launch_arguments={
            'config': 'zephyrus_real',
            'config_path': ov_config_path,
            'use_sim_time': 'false',
            'use_stereo': 'false',
            'max_cameras': '1',
            'rviz_enable': 'false',
        }.items(),
        condition=launch.conditions.IfCondition(LaunchConfiguration('use_vio'))
    )

    use_vicon_val = PythonExpression([
        "'false' if '", LaunchConfiguration('sitl'), "' == 'true' else '", LaunchConfiguration('use_vicon'), "'"
    ])

    vicon_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('vicon_receiver'), 'launch', 'client.launch.py']
            )
        ),
        launch_arguments={'hostname': '192.168.0.78'}.items(),
        condition=launch.conditions.IfCondition(use_vicon_val)
    )

    # --- 4.1 EKF: State Estimation ---
    vicon_topic = PythonExpression([
        "'/vicon/' + '", LaunchConfiguration('vicon_subject'), "' + '/' + '", LaunchConfiguration('vicon_segment'), "'"
    ])
    
    ekf_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ekf'), 'launch', 'rl_ekf.launch.xml'])
        ),
        launch_arguments={
            'pose1': vicon_topic
        }.items(),
        condition=launch.conditions.IfCondition(LaunchConfiguration('use_ekf'))
    )

    # --- 4.2 STATE BRIDGE: Odometry -> Float64MultiArray ---
    state_bridge_node = Node(
        package='drone_control',
        executable='state_bridge_node',
        name='state_estimator_bridge',
        parameters=[{
            'pose_source': PythonExpression(["'odometry' if '", LaunchConfiguration('sitl'), "' == 'true' else 'vicon'"]),
            'vicon_topic': vicon_topic,
            'odometry_topic': '/odometry',
            'imu_topic': '/imu0',
            'state_topic': 'drone_state_sim',
            'publish_rate': 100.0
        }],
        output='screen'
    )

    # --- 5. CONTROL: Drone Controller ---
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_drone_control, 'launch', 'drone_controller.launch.py')
        ),
        launch_arguments={
            'plotjuggler': 'false',
            'use_mavsdk': 'false',
            'controller_type': LaunchConfiguration('controller_type'),
            'rate_error_scale': LaunchConfiguration('rate_error_scale')
        }.items() 
    )

    # --- 6. SITL SPECIFIC LAUNCHES ---
    gazebo_world_path = os.path.join(ws_root, 'sim', 'gazebo', 'worlds', 'sitl_racing.sdf')
    
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={
            'gz_args': [gazebo_world_path, ' -s -r'],
            'gz_version': '8'
        }.items(),
        condition=launch.conditions.IfCondition(LaunchConfiguration('sitl'))
    )

    gz_bridges_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros_bridges'),
                'launch',
                'bridge.launch.py'
            ])
        ),
        condition=launch.conditions.IfCondition(LaunchConfiguration('sitl'))
    )

    bf_sitl_bin = os.path.join(ws_root, 'ext', 'anemos-betaflight', 'obj', 'main', 'betaflight_SITL.elf')
    betaflight_sitl_process = ExecuteProcess(
        cmd=[bf_sitl_bin],
        cwd=ws_root,
        output='screen',
        condition=launch.conditions.IfCondition(LaunchConfiguration('sitl'))
    )

    sitl_rc_bridge_node = Node(
        package='betaflight_msp_controller',
        executable='sitl_rc_bridge',
        name='sitl_rc_bridge',
        parameters=[{
            'attitude_cmd_topic': LaunchConfiguration('attitude_cmd_topic'),
            'sitl_ip': '127.0.0.1',
            'sitl_rc_port': 9004
        }],
        condition=launch.conditions.IfCondition(LaunchConfiguration('sitl')),
        output='screen'
    )

    return LaunchDescription([
        arg_sitl,
        arg_imu_port,
        arg_imu_source,
        arg_imu_baud,
        arg_msp_port,
        arg_attitude_topic,
        arg_target_hz,
        arg_use_vicon,
        arg_vicon_subject,
        arg_vicon_segment,
        arg_use_rc_sim,
        arg_view_stream,
        arg_video_bitrate,
        arg_video_width,
        arg_video_height,
        arg_video_fps,
        arg_video_port,
        arg_h264_backend,
        arg_isaac_h264_qp,
        arg_isaac_h264_iframe_interval,
        arg_isaac_h264_profile,
        arg_isaac_h264_hw_preset,
        arg_decode_h264,
        arg_ground_station_ip,
        arg_publish_raw,
        arg_publish_rgb,
        arg_camera_info_url,
        arg_use_gate_detection,
        arg_gate_camera_topic,
        arg_use_vio,
        arg_use_ekf,
        arg_msp_bench_mode,
        arg_load_bf_config,
        arg_controller_type,
        arg_rate_error_scale,
        blackbox_node,
        msp_imu_node,
        msp_bridge_node,
        bench_mode_warning,
        load_config_process,
        video_publisher_node,
        gstreamer_stream_enabled_for_perception,
        gstreamer_ros_h264_enabled,
        isaac_h264_encoder,
        isaac_h264_enabled,
        video_receiver,
        isaac_h264_decoder,
        camera_launch,
        gate_detection_launch,
        vio_launch,
        vicon_launch,
        ekf_launch,
        state_bridge_node,
        control_launch,
        gazebo_launch,
        gz_bridges_launch,
        betaflight_sitl_process,
        sitl_rc_bridge_node,
    ])
