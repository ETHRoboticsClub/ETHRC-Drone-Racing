import os
import socket
import launch.conditions

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # --- Machine Detection ---
    hostname = socket.gethostname()
    is_jetson = hostname in ['zephyrus', 'boreas', 'jetson']
    
    # --- Packages ---
    pkg_drone_control = get_package_share_directory('drone_control')

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
    arg_view_stream = DeclareLaunchArgument('view_stream', default_value='false',
                                               description='Start the secondary RTP/H.264 video path')
    
    # Video defaults: 720p @ 60fps is optimized for racing
    arg_video_bitrate = DeclareLaunchArgument('video_bitrate', default_value='4000000')
    arg_video_width = DeclareLaunchArgument('video_width', default_value='1280')
    arg_video_height = DeclareLaunchArgument('video_height', default_value='720')
    arg_video_fps = DeclareLaunchArgument('video_fps', default_value='60')
    arg_video_port = DeclareLaunchArgument('video_port', default_value='5000')
    arg_h264_backend = DeclareLaunchArgument(
        'h264_backend', default_value='isaac_ros',
        description='H.264 backend when view_stream=true: isaac_ros or gstreamer'
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
        'ground_station_ip', default_value='10.200.117.1',
        description='Ground station IPv4 address for the Jetson RTP/H.264 UDP stream'
    )
    arg_publish_raw = DeclareLaunchArgument(
        'publish_raw', default_value='true',
        description='Publish /cam0/image_raw for VIO and gate detection'
    )
    arg_use_gate_detection = DeclareLaunchArgument(
        'use_gate_detection', default_value='false',
        description='Gate detector is WIP; enable only after gate_detection is built and installed'
    )
    arg_use_vio = DeclareLaunchArgument('use_vio', default_value='true')

    # Bench Mode Arguments from Main
    arg_msp_bench_mode = DeclareLaunchArgument('msp_bench_mode', default_value='false',
                                               description='Disable MSP Bridge to allow standalone TX emulator')
    arg_load_bf_config = DeclareLaunchArgument('load_bf_config', default_value='false',
                                               description='Automatically load Betaflight config on startup')
    
    # Paths to config files (Bench Mode)
    ws_root = os.path.abspath(os.path.join(pkg_drone_control, '../../../../'))
    bench_config_path = os.path.join(ws_root, 'docs', 'docs', 'files', 'betaflight_conf', 'ZEPHYRUS_MSP_BENCH.txt')
    flight_config_path = os.path.join(ws_root, 'docs', 'docs', 'files', 'betaflight_conf', 'zephyrus_KAKUTEH7.txt')

    # --- 1. IMU DRIVER: Betaflight Blackbox ---
    blackbox_node = Node(
        package='betaflight_blackbox_driver',
        executable='blackbox_driver_node',
        name='imu_driver_node',
        parameters=[{
            'port': LaunchConfiguration('imu_port'),
            'baud': 1500000,
            'imu_topic': '/imu0',
            'enable_msp_control': False, # Explicitly disabled in separate port mode
            'motor_poles': 14,
            'rpm_topic': '/motor_rpm',
        }],
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
    selected_config_path = PythonExpression([
        "'", bench_config_path, "' if '", LaunchConfiguration('msp_bench_mode'), "' == 'true' else '", flight_config_path, "'"
    ])
    
    load_config_process = ExecuteProcess(
        cmd=['ros2', 'run', 'betaflight_msp_controller', 'load_bf_config', 
             selected_config_path, '--port', LaunchConfiguration('msp_port')],
        output='screen',
        condition=launch.conditions.IfCondition(LaunchConfiguration('load_bf_config'))
    )

    # --- 3. VIDEO: Team-Standard Camera for perception ---
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('imx219_argus_camera'), 'launch', 'isaac_camera.launch.py']
            )
        ),
        launch_arguments={
            'sensor_id': '0',
            'sensor_mode': '4',
            'framerate': LaunchConfiguration('video_fps'),
            'width': LaunchConfiguration('video_width'),
            'height': LaunchConfiguration('video_height'),
            'publish_color': PythonExpression([
                "'", LaunchConfiguration('view_stream'), "' == 'true' and '",
                LaunchConfiguration('h264_backend'), "' == 'isaac_ros'"
            ]),
            'color_encoding': 'rgb8',
            'enable_h264_stream': PythonExpression([
                "'", LaunchConfiguration('view_stream'), "' == 'true' and '",
                LaunchConfiguration('h264_backend'), "' == 'gstreamer'"
            ]),
            'ground_station_ip': LaunchConfiguration('ground_station_ip'),
            'stream_port': LaunchConfiguration('video_port'),
            'stream_bitrate': LaunchConfiguration('video_bitrate'),
        }.items(),
        condition=launch.conditions.IfCondition(
            PythonExpression(["'", LaunchConfiguration('publish_raw'), "' == 'true' and ", str(is_jetson)])
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

    isaac_h264_encoder = GroupAction(
        actions=[
            SetRemap(src='image_raw', dst='/cam0/image_rgb'),
            SetRemap(src='image_compressed', dst='/cam0/image_h264'),
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
                    'config': 'custom',
                    'hw_preset': LaunchConfiguration('isaac_h264_hw_preset'),
                    'profile': LaunchConfiguration('isaac_h264_profile'),
                    'iframe_interval': LaunchConfiguration('isaac_h264_iframe_interval'),
                    'qp': LaunchConfiguration('isaac_h264_qp'),
                }.items(),
            ),
        ],
        condition=launch.conditions.IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('view_stream'), "' == 'true' and '",
                LaunchConfiguration('publish_raw'), "' == 'true' and '",
                LaunchConfiguration('h264_backend'), "' == 'isaac_ros' and ", str(is_jetson)
            ])
        )
    )

    isaac_h264_enabled = LogInfo(
        msg="Isaac ROS H.264 encoder enabled: /cam0/image_rgb -> /cam0/image_h264. View /image_decoded after decoding; /cam0/image_h264 is compressed bytes, not a directly displayable image.",
        condition=launch.conditions.IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('view_stream'), "' == 'true' and '",
                LaunchConfiguration('publish_raw'), "' == 'true' and '",
                LaunchConfiguration('h264_backend'), "' == 'isaac_ros' and ", str(is_jetson)
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

    isaac_h264_decoder = GroupAction(
        actions=[
            SetRemap(src='image_compressed', dst='/cam0/image_h264'),
            SetRemap(src='image_uncompressed', dst='/image_decoded'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('isaac_ros_h264_decoder'),
                        'launch',
                        'isaac_ros_h264_decoder.launch.py'
                    ])
                )
            ),
        ],
        condition=launch.conditions.IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('view_stream'), "' == 'true' and '",
                LaunchConfiguration('h264_backend'), "' == 'isaac_ros' and (not ", str(is_jetson),
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
            'camera_topic': '/cam0/image_raw',
            'output_topic': '/gate_detection/drone_pose',
        }.items(),
        condition=launch.conditions.IfCondition(LaunchConfiguration('use_gate_detection'))
    )

    ov_config_path = PathJoinSubstitution(
        [FindPackageShare('ov_msckf'), 'config', 'zephyrus_imx219_720p', 'estimator_config.yaml']
    )
    vio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('ov_msckf'), 'launch', 'subscribe.launch.py']
            )
        ),
        launch_arguments={
            'config': 'zephyrus_imx219_720p',
            'config_path': ov_config_path,
            'use_sim_time': 'false',
            'use_stereo': 'false',
            'max_cameras': '1',
            'rviz_enable': 'false',
        }.items(),
        condition=launch.conditions.IfCondition(LaunchConfiguration('use_vio'))
    )

    # --- 4. VICON: Ground Truth ---
    vicon_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('vicon_receiver'), 'launch', 'client.launch.py']
            )
        ),
        launch_arguments={'hostname': '10.200.117.1'}.items(),
        condition=launch.conditions.IfCondition(LaunchConfiguration('use_vicon'))
    )

    # --- 5. CONTROL: Drone Controller ---
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
        arg_video_port,
        arg_h264_backend,
        arg_isaac_h264_qp,
        arg_isaac_h264_iframe_interval,
        arg_isaac_h264_profile,
        arg_isaac_h264_hw_preset,
        arg_decode_h264,
        arg_ground_station_ip,
        arg_publish_raw,
        arg_use_gate_detection,
        arg_use_vio,
        arg_msp_bench_mode,
        arg_load_bf_config,
        blackbox_node,
        msp_bridge_node,
        bench_mode_warning,
        load_config_process,
        video_publisher_node,
        gstreamer_stream_enabled_for_perception,
        isaac_h264_encoder,
        isaac_h264_enabled,
        video_receiver,
        isaac_h264_decoder,
        camera_launch,
        gate_detection_launch,
        vio_launch,
        vicon_launch,
        control_launch,
    ])
