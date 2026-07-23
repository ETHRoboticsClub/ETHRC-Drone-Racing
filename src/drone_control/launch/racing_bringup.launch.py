import os
import socket
import errno
import launch.conditions

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import (
    EmitEvent,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare


def _launch_bool(value):
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _proc_cmdline(pid):
    try:
        with open(f'/proc/{pid}/cmdline', 'rb') as f:
            raw = f.read().replace(b'\0', b' ').strip()
        return raw.decode('utf-8', errors='replace')
    except OSError:
        return ''


def _current_process_family():
    pids = set()
    pid = os.getpid()
    while pid and pid not in pids:
        pids.add(pid)
        try:
            with open(f'/proc/{pid}/stat', 'r', encoding='utf-8') as f:
                fields = f.read().split()
            pid = int(fields[3])
        except (OSError, IndexError, ValueError):
            break
    return pids


def _matching_processes(patterns):
    matches = []
    ignored = _current_process_family()

    try:
        proc_entries = os.listdir('/proc')
    except OSError:
        return matches

    for name in proc_entries:
        if not name.isdigit():
            continue
        pid = int(name)
        if pid in ignored:
            continue

        cmdline = _proc_cmdline(pid)
        if not cmdline:
            continue

        if any(pattern in cmdline for pattern in patterns):
            matches.append((pid, cmdline))

    return matches


def _port_is_bound(host, port, socket_type):
    sock = socket.socket(socket.AF_INET, socket_type)
    try:
        sock.bind((host, port))
        return False
    except OSError as exc:
        if exc.errno in (errno.EADDRINUSE, errno.EACCES):
            return True
        raise
    finally:
        sock.close()


def _sitl_preflight(context, ws_root, bf_sitl_bin, sitl_plugins_dir):
    if not _launch_bool(LaunchConfiguration('preflight').perform(context)):
        return []

    if not _launch_bool(LaunchConfiguration('sitl').perform(context)):
        return []

    errors = []
    warnings = []

    world_path = LaunchConfiguration('sitl_world').perform(context)
    if not os.path.isfile(world_path):
        errors.append(f'SITL world does not exist: {world_path}')

    if not os.path.isfile(bf_sitl_bin):
        errors.append(f'Betaflight SITL binary is missing: {bf_sitl_bin}')
    elif not os.access(bf_sitl_bin, os.X_OK):
        errors.append(f'Betaflight SITL binary is not executable: {bf_sitl_bin}')

    plugin_path = os.path.join(sitl_plugins_dir, 'libEthrcBetaflightSITLPlugin.so')
    if not os.path.isfile(plugin_path):
        errors.append(
            'EthrcBetaflightSITLPlugin is missing. Build it with: '
            'colcon build --packages-select sitl_gazebo_plugins'
        )

    boreas_config = os.path.join(ws_root, 'config', 'airframes', 'boreas_4s.yaml')
    if not os.path.isfile(boreas_config):
        errors.append(f'Boreas airframe configuration is missing: {boreas_config}')

    if not os.path.isfile(os.path.join(ws_root, 'eeprom.bin')):
        warnings.append('eeprom.bin is missing; run scripts/setup_sitl.sh before SITL if Betaflight config is not loaded yet.')

    process_patterns = [
        'betaflight_SITL.elf',
        'sitl_rc_bridge',
        'parameter_bridge /drone/camera',
        'gz-sim-main',
        ' gz sim ',
        'drone_controller_node',
        'state_bridge_node',
    ]
    process_matches = _matching_processes(process_patterns)
    if process_matches:
        details = ', '.join(f'{pid}: {cmd[:120]}' for pid, cmd in process_matches[:8])
        extra = '' if len(process_matches) <= 8 else f' (+{len(process_matches) - 8} more)'
        errors.append(
            'Another SITL/racing bringup process appears to be running: '
            f'{details}{extra}. Stop the stale process with Ctrl-C before launching again.'
        )

    port_checks = [
        ('tcp', 5761, 'Betaflight UART1/MSP TCP'),
        ('udp', 9001, 'Betaflight RealFlight output'),
        ('udp', 9002, 'Gazebo motor input'),
        ('udp', 9003, 'Betaflight FDM input'),
        ('udp', 9004, 'Betaflight RC input'),
        ('udp', 9010, 'Joy emulator UDP fallback'),
    ]
    for protocol, port, label in port_checks:
        socket_type = socket.SOCK_STREAM if protocol == 'tcp' else socket.SOCK_DGRAM
        try:
            if _port_is_bound('127.0.0.1', port, socket_type):
                errors.append(f'Port {protocol.upper()} 127.0.0.1:{port} is already in use ({label}).')
        except OSError as exc:
            warnings.append(f'Could not check {protocol.upper()} 127.0.0.1:{port}: {exc}')

    if os.environ.get('WSL_INTEROP') or _running_under_wsl():
        rmw = os.environ.get('RMW_IMPLEMENTATION', '')
        discovery = os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE', '')
        fastdds_profile = os.environ.get('FASTDDS_DEFAULT_PROFILES_FILE', '')
        if rmw == 'rmw_fastrtps_cpp' and discovery == 'LOCALHOST' and not fastdds_profile:
            warnings.append(
                'WSL is using FastDDS with LOCALHOST discovery and no FASTDDS_DEFAULT_PROFILES_FILE. '
                'Run: source scripts/source_sitl_env.sh'
            )
        if fastdds_profile and not os.path.isfile(fastdds_profile):
            errors.append(f'FASTDDS_DEFAULT_PROFILES_FILE points to a missing file: {fastdds_profile}')

    if warnings:
        print('[racing_bringup_health] WARNINGS:')
        for warning in warnings:
            print(f'  - {warning}')

    if not errors:
        print('[racing_bringup_health] SITL preflight passed.')
        return []

    print('[racing_bringup_health] ERRORS:')
    for error in errors:
        print(f'  - {error}')

    raise RuntimeError('Racing bringup preflight failed; fix the errors above or pass preflight:=false to bypass.')


def _running_under_wsl():
    if os.environ.get('WSL_INTEROP') or os.environ.get('WSL_DISTRO_NAME'):
        return True
    try:
        with open('/proc/sys/kernel/osrelease', 'r', encoding='utf-8') as f:
            return 'microsoft' in f.read().lower()
    except OSError:
        return False


def _detect_primary_ipv4():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect(('1.1.1.1', 80))
        return sock.getsockname()[0]
    except OSError:
        return ''
    finally:
        sock.close()


def _write_fastdds_wsl_profile(ip_addr):
    if not ip_addr:
        return ''

    profile_path = os.environ.get(
        'ETHRC_FASTDDS_WSL_PROFILE',
        f"/tmp/ethrc_fastdds_wsl_{os.getuid()}.xml"
    )
    try:
        os.makedirs(os.path.dirname(profile_path), exist_ok=True)
        with open(profile_path, 'w', encoding='utf-8') as f:
            f.write(f'''<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>ethrc_wsl_udp</transport_id>
      <type>UDPv4</type>
      <interfaceWhiteList>
        <address>{ip_addr}</address>
      </interfaceWhiteList>
    </transport_descriptor>
  </transport_descriptors>

  <participant profile_name="ros2_default_profile" is_default_profile="true">
    <rtps>
      <useBuiltinTransports>false</useBuiltinTransports>
      <userTransports>
        <transport_id>ethrc_wsl_udp</transport_id>
      </userTransports>
    </rtps>
  </participant>
</profiles>
''')
        return profile_path
    except OSError:
        return ''


def _configure_wsl_fastdds_defaults():
    if not _running_under_wsl():
        return
    if os.environ.get('ETHRC_DISABLE_WSL_FASTDDS_PROFILE') == '1':
        return
    if os.environ.get('FASTDDS_DEFAULT_PROFILES_FILE'):
        return
    if os.environ.get('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp') != 'rmw_fastrtps_cpp':
        return

    profile_path = _write_fastdds_wsl_profile(_detect_primary_ipv4())
    if not profile_path:
        return

    os.environ['FASTDDS_DEFAULT_PROFILES_FILE'] = profile_path
    os.environ.setdefault('RMW_FASTRTPS_USE_QOS_FROM_XML', '1')
    if os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE', '') in ('', 'LOCALHOST'):
        os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'SUBNET'


def generate_launch_description():
    # --- Machine Detection ---
    hostname = socket.gethostname()
    is_jetson = hostname in ['zephyrus', 'boreas', 'jetson']
    
    # --- Packages ---
    pkg_drone_control = get_package_share_directory('drone_control')
    ws_root = os.path.abspath(os.path.join(pkg_drone_control, '../../../../'))

    # --- Paths for SITL ---
    os.environ.setdefault('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')
    os.environ.setdefault('GALLIUM_DRIVER', 'd3d12')
    os.environ.setdefault('LIBGL_ALWAYS_SOFTWARE', '0')
    os.environ.setdefault('QT_QPA_PLATFORM', 'xcb')
    _configure_wsl_fastdds_defaults()
    # WSL often exposes Docker / virtual switch interfaces that Gazebo Transport
    # may pick before loopback. Keep Gazebo Transport on localhost; ROS DDS is
    # handled separately by scripts/source_sitl_env.sh when running under WSL.
    os.environ.setdefault('GZ_IP', '127.0.0.1')
    gz_config_path = os.environ.get('GZ_CONFIG_PATH', '')
    if '/usr/share/gz' not in gz_config_path:
        os.environ['GZ_CONFIG_PATH'] = f"/usr/share/gz:{gz_config_path}" if gz_config_path else '/usr/share/gz'
    models_dir = os.path.join(ws_root, 'sim', 'gazebo', 'models')
    gz_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    if models_dir not in gz_resource_path:
        os.environ['GZ_SIM_RESOURCE_PATH'] = f"{models_dir}:{gz_resource_path}" if gz_resource_path else models_dir

    plugins_dir = os.path.join(ws_root, 'ext', 'aeroloop_gazebo', 'plugins', 'build')
    sitl_plugins_dir = os.path.join(ws_root, 'install', 'sitl_gazebo_plugins', 'lib')
    gz_plugin_path = os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
    if sitl_plugins_dir not in gz_plugin_path:
        os.environ['GZ_SIM_SYSTEM_PLUGIN_PATH'] = f"{sitl_plugins_dir}:{gz_plugin_path}" if gz_plugin_path else sitl_plugins_dir
        gz_plugin_path = os.environ['GZ_SIM_SYSTEM_PLUGIN_PATH']
    if plugins_dir not in gz_plugin_path:
        os.environ['GZ_SIM_SYSTEM_PLUGIN_PATH'] = f"{plugins_dir}:{gz_plugin_path}" if gz_plugin_path else plugins_dir

    # --- Arguments ---
    arg_sitl = DeclareLaunchArgument('sitl', default_value='false',
                                     description='Whether to run in SITL mode with Gazebo and Betaflight')
    arg_preflight = DeclareLaunchArgument(
        'preflight',
        default_value='true',
        description='Run SITL health checks before starting Gazebo, Betaflight, and bridge processes'
    )
    default_sitl_world = os.path.join(ws_root, 'sim', 'gazebo', 'worlds', 'sitl_racing_custom.sdf')
    arg_sitl_world = DeclareLaunchArgument(
        'sitl_world',
        default_value=default_sitl_world,
        description='Gazebo world SDF to launch when sitl:=true'
    )

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
        'h264_backend', default_value='gstreamer_ros',
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
        'publish_raw', default_value='true',
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
                                              choices=['rl', 'mpc'],
                                              description='Type of controller: "mpc" or "rl"')
    arg_rate_error_scale = DeclareLaunchArgument('rate_error_scale', default_value='300.0',
                                               description='Torque-to-rate scaling for RL controller')
    arg_rl_default_goal_enabled = DeclareLaunchArgument(
        'rl_default_goal_enabled',
        default_value='true',
        description='Seed a default hover goal when running SITL with the RL controller'
    )
    arg_rl_default_goal_x = DeclareLaunchArgument('rl_default_goal_x', default_value='0.0')
    arg_rl_default_goal_y = DeclareLaunchArgument('rl_default_goal_y', default_value='0.0')
    arg_rl_default_goal_z = DeclareLaunchArgument('rl_default_goal_z', default_value='1.5')

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
            # A simulated camera/IMU pair is only meaningful when OpenVINS
            # consumes the same Gazebo clock epoch as its measurements.
            # Hardware retains wall-clock time.
            'use_sim_time': LaunchConfiguration('sitl'),
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
            'rate_error_scale': LaunchConfiguration('rate_error_scale'),
            'default_goal_enabled': PythonExpression([
                "'true' if '", LaunchConfiguration('sitl'), "' == 'true' and '",
                LaunchConfiguration('controller_type'), "' in ('rl', 'mpc') and '",
                LaunchConfiguration('rl_default_goal_enabled'), "' == 'true' else 'false'"
            ]),
            'default_goal_x': LaunchConfiguration('rl_default_goal_x'),
            'default_goal_y': LaunchConfiguration('rl_default_goal_y'),
            'default_goal_z': LaunchConfiguration('rl_default_goal_z'),
        }.items() 
    )

    # --- 6. SITL SPECIFIC LAUNCHES ---
    gazebo_process = ExecuteProcess(
        cmd=['gz', 'sim', LaunchConfiguration('sitl_world'), '-s', '-r', '--force-version', '8'],
        additional_env={
            'ETHRC_BOREAS_CONFIG': os.path.join(
                ws_root, 'config', 'airframes', 'boreas_4s.yaml'
            ),
        },
        condition=launch.conditions.IfCondition(LaunchConfiguration('sitl'))
    )
    gazebo_exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=gazebo_process,
            on_exit=[
                LogInfo(msg='Gazebo server exited; shutting down racing bringup.'),
                EmitEvent(event=Shutdown(reason='Gazebo server exited')),
            ],
        )
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
    sitl_preflight = OpaqueFunction(
        function=lambda context: _sitl_preflight(context, ws_root, bf_sitl_bin, sitl_plugins_dir)
    )

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
            'sitl_rc_port': 9004,
            'default_override': PythonExpression([
                "'true' if '", LaunchConfiguration('controller_type'), "' in ('rl', 'mpc') else 'false'"
            ]),
            'lock_default_override': PythonExpression([
                "'true' if '", LaunchConfiguration('controller_type'), "' in ('rl', 'mpc') else 'false'"
            ]),
        }],
        condition=launch.conditions.IfCondition(LaunchConfiguration('sitl')),
        output='screen'
    )

    sitl_discovery_range = 'SUBNET' if os.environ.get('FASTDDS_DEFAULT_PROFILES_FILE') else 'LOCALHOST'

    return LaunchDescription([
        arg_sitl,
        arg_preflight,
        arg_sitl_world,
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
        arg_rl_default_goal_enabled,
        arg_rl_default_goal_x,
        arg_rl_default_goal_y,
        arg_rl_default_goal_z,
        SetEnvironmentVariable(
            name='RMW_IMPLEMENTATION',
            value=PythonExpression([
                "'rmw_fastrtps_cpp' if '", LaunchConfiguration('sitl'), "' == 'true' else '",
                os.environ.get('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'), "'"
            ])
        ),
        SetEnvironmentVariable(
            name='ROS_AUTOMATIC_DISCOVERY_RANGE',
            value=PythonExpression([
                "'", sitl_discovery_range, "' if '",
                LaunchConfiguration('sitl'), "' == 'true' else '",
                os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE', 'SUBNET'), "'"
            ])
        ),
        sitl_preflight,
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
        gazebo_process,
        gazebo_exit_handler,
        gz_bridges_launch,
        betaflight_sitl_process,
        sitl_rc_bridge_node,
    ])
