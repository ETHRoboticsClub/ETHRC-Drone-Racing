from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim = LaunchConfiguration("use_sim")
    use_camera = LaunchConfiguration("use_camera")
    use_gate_detection = LaunchConfiguration("use_gate_detection")
    use_vio = LaunchConfiguration("use_vio")
    use_ekf = LaunchConfiguration("use_ekf")
    use_video_stream = LaunchConfiguration("use_video_stream")
    h264_backend = LaunchConfiguration("h264_backend")
    gate_camera_topic = LaunchConfiguration("gate_camera_topic")
    camera_info_url = LaunchConfiguration("camera_info_url")

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("drone_control"), "launch", "nitros_camera.launch.py"]
            )
        ),
        launch_arguments={
            "sensor_id": "0",
            "sensor_mode": "4",
            "image_width": LaunchConfiguration("camera_width"),
            "image_height": LaunchConfiguration("camera_height"),
            "framerate": LaunchConfiguration("camera_fps"),
            "publish_mono": "true",
            "publish_rgb": LaunchConfiguration("publish_rgb"),
            "camera_info_url": camera_info_url,
        }.items(),
        condition=IfCondition(
            PythonExpression(["'", use_sim, "' == 'false' and '", use_camera, "' == 'true'"])
        ),
    )

    camera_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("gazebo_ros_bridges"), "launch", "bridge.launch.py"]
            )
        ),
        condition=IfCondition(use_sim),
    )

    gate_detection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("gate_detection"), "launch", "gate_detection.launch.py"]
            )
        ),
        launch_arguments={
            "camera_topic": gate_camera_topic,
            "output_topic": "/gate_detection/drone_pose",
        }.items(),
        condition=IfCondition(use_gate_detection),
    )

    ov_config_path = PathJoinSubstitution(
        [FindPackageShare("ov_msckf"), "config", "zephyrus_imx219_720p", "estimator_config.yaml"]
    )
    vio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ov_msckf"), "launch", "subscribe.launch.py"])
        ),
        launch_arguments={
            "config": "zephyrus_imx219_720p",
            "config_path": ov_config_path,
            "use_sim_time": use_sim,
            "use_stereo": "false",
            "max_cameras": "1",
            "rviz_enable": "false",
        }.items(),
        condition=IfCondition(use_vio),
    )

    ekf_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ekf"), "launch", "rl_ekf.launch.xml"])
        ),
        condition=IfCondition(use_ekf),
    )

    h264_encoder_launch = GroupAction(
        actions=[
            SetRemap(src="image_raw", dst="/cam0/image_raw_nitros"),
            SetRemap(src="image_compressed", dst="/cam0/image_h264"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("isaac_ros_h264_encoder"), "launch", "isaac_ros_h264_encoder.launch.py"]
                    )
                ),
                launch_arguments={
                    "input_width": LaunchConfiguration("camera_width"),
                    "input_height": LaunchConfiguration("camera_height"),
                    "config": "custom",
                    "hw_preset": LaunchConfiguration("isaac_h264_hw_preset"),
                    "profile": LaunchConfiguration("isaac_h264_profile"),
                    "iframe_interval": LaunchConfiguration("isaac_h264_iframe_interval"),
                    "qp": LaunchConfiguration("isaac_h264_qp"),
                }.items(),
            ),
        ],
        condition=IfCondition(
            PythonExpression([
                "'", use_video_stream, "' == 'true' and '",
                use_camera, "' == 'true' and '", use_sim, "' == 'false' and '",
                h264_backend, "' == 'isaac_ros'",
            ])
        ),
    )

    h264_encoder_enabled = LogInfo(
        msg="Isaac ROS H.264 encoder enabled: /cam0/image_raw_nitros -> /cam0/image_h264.",
        condition=IfCondition(
            PythonExpression([
                "'", use_video_stream, "' == 'true' and '",
                use_camera, "' == 'true' and '", use_sim, "' == 'false' and '",
                h264_backend, "' == 'isaac_ros'",
            ])
        ),
    )

    gstreamer_ros_h264_enabled = LogInfo(
        msg="GStreamer ROS H.264 encoder enabled inside the camera node: /cam0/image_h264.",
        condition=IfCondition(
            PythonExpression([
                "'", use_video_stream, "' == 'true' and '",
                use_camera, "' == 'true' and '", use_sim, "' == 'false' and '",
                h264_backend, "' == 'gstreamer_ros'",
            ])
        ),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim", default_value="false"),
            DeclareLaunchArgument("use_camera", default_value="true"),
            DeclareLaunchArgument("use_gate_detection", default_value="true"),
            DeclareLaunchArgument("use_vio", default_value="true"),
            DeclareLaunchArgument("use_ekf", default_value="true"),
            DeclareLaunchArgument("use_video_stream", default_value="true"),
            DeclareLaunchArgument("publish_rgb", default_value="true"),
            DeclareLaunchArgument("gate_camera_topic", default_value="/cam0/image_rgb"),
            DeclareLaunchArgument("h264_backend", default_value="isaac_ros"),
            DeclareLaunchArgument("camera_width", default_value="1280"),
            DeclareLaunchArgument("camera_height", default_value="720"),
            DeclareLaunchArgument("camera_fps", default_value="60"),
            DeclareLaunchArgument("ground_station_ip", default_value="10.200.117.1"),
            DeclareLaunchArgument("video_port", default_value="5000"),
            DeclareLaunchArgument("isaac_h264_qp", default_value="18"),
            DeclareLaunchArgument("isaac_h264_iframe_interval", default_value="30"),
            DeclareLaunchArgument("isaac_h264_profile", default_value="1"),
            DeclareLaunchArgument("isaac_h264_hw_preset", default_value="0"),
            DeclareLaunchArgument(
                "camera_info_url",
                default_value="file:///home/aero/proj/ETHRC-Drone-Racing/src/perception/camera_calibration/camera_info.yaml",
                description="Path to standard camera calibration YAML file"
            ),
            camera_launch,
            camera_bridge_launch,
            gate_detection_launch,
            vio_launch,
            ekf_launch,
            h264_encoder_launch,
            h264_encoder_enabled,
            gstreamer_ros_h264_enabled,
        ]
    )
