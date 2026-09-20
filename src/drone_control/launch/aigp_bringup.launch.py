import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_drone_control = get_package_share_directory('drone_control')
    
    # Path to AIGP camera driver configurations
    camera_config = PathJoinSubstitution([
        FindPackageShare('aigp_camera_driver'), 'config', 'camera_params.yaml'
    ])
    default_policy_path = PathJoinSubstitution([
        FindPackageShare('rl'), 'models', 'nicola', 'policy.pt'
    ])
    default_track_path = PathJoinSubstitution([
        FindPackageShare('rl'), 'config', 'aigp_competition_track.yaml'
    ])
    use_aigp_policy = PythonExpression([
        "'", LaunchConfiguration('controller_type'), "' == 'aigp_policy'"
    ])
    use_vq1_gt = PythonExpression([
        "'", LaunchConfiguration('controller_type'), "' == 'aigp_vq1_gt'"
    ])
    use_teleop = PythonExpression([
        "'", LaunchConfiguration('controller_type'), "' == 'teleop'"
    ])
    use_estimator_pipeline = PythonExpression([
        "'", LaunchConfiguration('controller_type'), "' in ['aigp_policy', 'teleop'] and '",
        LaunchConfiguration('state_source'), "' == 'estimator'"
    ])
    policy_state_topic = PythonExpression([
        "'/aigp/ground_truth/state' if '", LaunchConfiguration('state_source'),
        "' == 'ground_truth' else '", LaunchConfiguration('state_topic'), "'"
    ])
    bridge_velocity_frame = PythonExpression([
        "'local_ned' if '", LaunchConfiguration('controller_type'),
        "' == 'aigp_vq1_gt' else '", LaunchConfiguration('velocity_command_frame'), "'"
    ])
    use_debug_visualizer = PythonExpression([
        "'", LaunchConfiguration('debug'), "' == 'true' or '", LaunchConfiguration('controller_type'), "' == 'teleop'"
    ])
    use_generic_controller = PythonExpression([
        "'", LaunchConfiguration('controller_type'), "' not in ['aigp_policy', 'aigp_vq1_gt', 'teleop']"
    ])

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument('controller_type', default_value='rl',
                              description='Type of controller: "mpc", "rl", "random", "teleop", "aigp_policy", or "aigp_vq1_gt"'),
        DeclareLaunchArgument('debug', default_value='false',
                              description='Enable real-time graphical debug dashboard and visualizer'),
        DeclareLaunchArgument('rate_error_scale', default_value='300.0',
                              description='Torque-to-rate scaling for RL controller'),
        DeclareLaunchArgument('rate_command_scale_rad_s', default_value='4.0',
                              description='Normalized rate scale for the AIGP TorchScript policy'),
        DeclareLaunchArgument('policy_rate_hz', default_value='60.0',
                              description='AIGP policy inference rate'),
        DeclareLaunchArgument('startup_hold_s', default_value='4.0',
                              description='Publish zero command for this long before policy output'),
        DeclareLaunchArgument('velocity_frame', default_value='world_ned',
                              description='Frame of estimated linear velocity for the AIGP policy'),
        DeclareLaunchArgument('velocity_limit', default_value='10.0',
                              description='Velocity limit value exposed to 105D policies'),
        DeclareLaunchArgument('observation_contract', default_value='auto',
                              description='Policy observation contract: auto, legacy_corners_25, or aigp_ours_105'),
        DeclareLaunchArgument('state_topic', default_value='/drone_state_estimated',
                              description='13-value NED/FRD state topic consumed by AIGP policy'),
        DeclareLaunchArgument('state_source', default_value='estimator',
                              description='Policy state source: "ground_truth" for VQ1 or "estimator" for VQ2'),
        DeclareLaunchArgument('state_estimator_rate_hz', default_value='100.0',
                              description='AIGP IMU/vision ESKF state publisher rate'),
        DeclareLaunchArgument('state_estimator_velocity_damping_per_s', default_value='0.0',
                              description='Optional estimator velocity damping (normally disabled)'),
        DeclareLaunchArgument('initial_yaw_rad', default_value='3.141592653589793',
                              description='Known fixed AIGP start yaw in NED radians'),
        DeclareLaunchArgument('policy_path', default_value=default_policy_path,
                              description='Path to Nicola TorchScript policy'),
        DeclareLaunchArgument('track_path', default_value=default_track_path,
                              description='Path to static AIGP competition track YAML'),
        DeclareLaunchArgument('device', default_value='udpin:0.0.0.0:14550',
                              description='MAVLink UDP connection string to AIGP simulator'),
        DeclareLaunchArgument('publish_imu', default_value='true',
                              description='Publish /imu0 from the bridge node for VIO'),
        DeclareLaunchArgument('auto_arm', default_value='false',
                              description='Arm automatically after heartbeat; keep false for shadow validation'),
        DeclareLaunchArgument('use_rad_per_second_body_rates', default_value='true',
                              description='Set the simulator-v4 type-mask extension for the attitude-target fallback'),
        DeclareLaunchArgument('rate_control_backend', default_value='actuator',
                              description='Body-rate backend: live-validated actuator or attitude_target fallback'),
        DeclareLaunchArgument('ground_truth_source', default_value='odometry',
                              description='Canonical VQ1 ground-truth source (odometry recommended)'),
        DeclareLaunchArgument('publish_imu_aux', default_value='false',
                              description='Publish raw HIGHRES_IMU magnetometer/pressure fields for sensor probing'),
        DeclareLaunchArgument('use_sim_timestamp', default_value='false',
                              description='Use raw simulator stamps (false keeps IMU/camera on local ROS time)'),
        DeclareLaunchArgument('velocity_command_frame', default_value='body_ned',
                              description='Frame for /drone_velocity_cmd: body_ned or local_ned'),
        DeclareLaunchArgument('vq1_controller_enabled', default_value='false',
                              description='Enable the VQ1 GT controller immediately (false permits shadow validation)'),
        DeclareLaunchArgument('vq1_target_speed_m_s', default_value='1.0',
                              description='Constant gate passage speed for the VQ1 GT controller'),
        
        # 1. AIGP MAVLink Bridge Node
        Node(
            package='betaflight_mavlink_bridge',
            executable='aigp_bridge_node',
            name='aigp_bridge_node',
            output='screen',
            parameters=[{
                'device': LaunchConfiguration('device'),
                'publish_imu': LaunchConfiguration('publish_imu'),
                'auto_arm': LaunchConfiguration('auto_arm'),
                'publish_imu_aux': LaunchConfiguration('publish_imu_aux'),
                'use_sim_timestamp': LaunchConfiguration('use_sim_timestamp'),
                'velocity_command_frame': bridge_velocity_frame,
                'use_rad_per_second_body_rates': LaunchConfiguration('use_rad_per_second_body_rates'),
                'ground_truth_source': LaunchConfiguration('ground_truth_source'),
                'rate_control_backend': LaunchConfiguration('rate_control_backend'),
            }]
        ),

        # 2. AIGP Camera Driver Composable Node Container
        ComposableNodeContainer(
            name='aigp_camera_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            output='screen',
            composable_node_descriptions=[
                ComposableNode(
                    package='aigp_camera_driver',
                    plugin='aigp_camera_driver::VisionDriverNode',
                    name='aigp_vision_driver_node',
                    parameters=[camera_config, {
                        'use_sim_timestamp': LaunchConfiguration('use_sim_timestamp'),
                    }],
                    remappings=[
                        ('image_raw', '/cam0/image_raw'),
                        ('image_rgb', '/cam0/image_rgb'),
                        ('camera_info', '/cam0/camera_info'),
                    ],
                    extra_arguments=[{'use_intra_process_comms': True}],
                )
            ]
        ),

        # 3a. IMU-only state estimator for AIGP simulator builds.
        Node(
            package='rl',
            executable='aigp_state_estimator_node',
            name='aigp_state_estimator_node',
            output='screen',
            condition=IfCondition(use_estimator_pipeline),
            parameters=[{
                'imu_topic': '/imu0',
                'state_topic': LaunchConfiguration('state_topic'),
                'publish_rate_hz': LaunchConfiguration('state_estimator_rate_hz'),
                'velocity_damping_per_s': LaunchConfiguration('state_estimator_velocity_damping_per_s'),
                'initial_yaw_rad': LaunchConfiguration('initial_yaw_rad'),
            }]
        ),

        # 3b. Nicola AIGP policy node.
        Node(
            package='rl',
            executable='aigp_policy_node',
            name='aigp_policy_node',
            output='screen',
            condition=IfCondition(use_aigp_policy),
            parameters=[{
                'policy_path': LaunchConfiguration('policy_path'),
                'track_path': LaunchConfiguration('track_path'),
                'policy_rate_hz': LaunchConfiguration('policy_rate_hz'),
                'rate_command_scale_rad_s': LaunchConfiguration('rate_command_scale_rad_s'),
                'startup_hold_s': LaunchConfiguration('startup_hold_s'),
                'velocity_frame': LaunchConfiguration('velocity_frame'),
                'velocity_limit': LaunchConfiguration('velocity_limit'),
                'observation_contract': LaunchConfiguration('observation_contract'),
                'state_topic': policy_state_topic,
            }]
        ),

        # 3b.2 Completion-first VQ1 controller. Ground truth closes a slow
        # gate-centering loop; the bridge's IMU-feedback actuator backend closes
        # body rates for the otherwise unknown simulator vehicle.
        Node(
            package='rl',
            executable='aigp_vq1_controller_node',
            name='aigp_vq1_controller_node',
            output='screen',
            condition=IfCondition(use_vq1_gt),
            parameters=[{
                'track_path': LaunchConfiguration('track_path'),
                'state_topic': '/aigp/ground_truth/state',
                'enabled': LaunchConfiguration('vq1_controller_enabled'),
                'target_speed_m_s': LaunchConfiguration('vq1_target_speed_m_s'),
            }]
        ),

        # 3c. Gate detector node for visual pose/position correction
        Node(
            package='gate_detection',
            executable='detector_node',
            name='gate_detector_node',
            output='screen',
            condition=IfCondition(use_estimator_pipeline),
            parameters=[{
                'track_path': LaunchConfiguration('track_path'),
                'camera_topic': '/cam0/image_rgb',
                'use_orange_detection': True,
            }]
        ),

        # 3d. Interactive Keyboard Teleop Node
        Node(
            package='drone_control',
            executable='teleop_keyboard_node',
            name='teleop_keyboard_node',
            output='screen',
            condition=IfCondition(use_teleop),
        ),

        # 3e. Graphical State Estimator Visualizer & Trajectory Dashboard Node
        Node(
            package='drone_control',
            executable='estimator_visualizer_node',
            name='estimator_visualizer_node',
            output='screen',
            condition=IfCondition(use_debug_visualizer),
            parameters=[{'state_topic': policy_state_topic}],
        ),

        # 3f. Generic controller launch for MPC/RL/random modes.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_drone_control, 'launch', 'drone_controller.launch.py')
            ),
            launch_arguments={
                'plotjuggler': 'false',
                'use_mavsdk': 'false',
                'controller_type': LaunchConfiguration('controller_type'),
                'rate_error_scale': LaunchConfiguration('rate_error_scale')
            }.items(),
            condition=IfCondition(use_generic_controller),
        )
    ])
