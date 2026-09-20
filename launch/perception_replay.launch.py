"""Replay-mode perception pipeline.

Mirrors ``launch/perception.launch.py`` but is purpose-built for offline
analysis of recorded ROS 2 bags (e.g. data_20260416/perception/<bag>/).

Differences from perception.launch.py
-------------------------------------
1. **No gazebo_ros_bridges** — the bag already has post-bridge topic names
   (`/cam0/image_raw`, `/cam0/camera_info`, `/imu0`).
2. **OpenVINS uses a `zephyrus_real` config** (untracked in the open_vins
   submodule). It's a copy of `gazebo_sim` with `init_dyn_use: true`
   flipped on, because the real-flight bag has the drone in motion from
   t=0 with no static phase — static-init configs (every other bundled
   one except `kaist`/`rpng_sim`, which have wrong topic names) just
   stall waiting for stillness. Real-VIO accuracy is broken anyway
   (camera↔IMU calibration still open), but for a wiring proof we only
   need OpenVINS to *publish*. Override via ``ov_config:=...`` if you
   have a proper real-hardware config.
3. **use_sim_time:=true everywhere** so /clock from `ros2 bag play --clock`
   drives the wall clock seen by EKF / OpenVINS / GateNet.
4. **EKF config is selectable** via ``ekf_config_file`` — defaults to the
   shipped 16-state quaternion config; pass `rl_ekf_legacy.yaml` to run
   the same launch against a legacy 15-state EKF build for A/B testing.

Note on camera intrinsics
-------------------------
This launch does NOT correct the bag's `/cam0/camera_info` topic, which
contains placeholder values (fx=fy=820, plumb_bob, no distortion). The
gate_detection package currently uses its own hardcoded intrinsics
(see [scripts/perception/replay/GATE_DETECTION_FINDINGS.md](../scripts/perception/replay/GATE_DETECTION_FINDINGS.md)
items 4 & 5) — that's a known content issue we don't address here. For
the wiring proof this launch is built for, the *flow* of messages is
what matters; the values would only matter for full perception accuracy.

End-to-end usage
----------------
    # Terminal A — start the perception stack
    ros2 launch launch/perception_replay.launch.py

    # Terminal B — play the bag
    ros2 bag play --clock \\
        ../data_20260416/perception/20260416_175153

    # Terminal C — record the perception outputs for analysis
    ros2 bag record -o out/replay_175153 \\
        /ekf/odometry /ov_msckf/odomimu /gate_detection/drone_pose \\
        /tf /tf_static /cam0/camera_info
"""
from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    # ------------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------------
    args = [
        DeclareLaunchArgument(
            "use_sim_time", default_value="true",
            description="Use /clock from `ros2 bag play --clock`"),
        DeclareLaunchArgument(
            "ekf_config_file", default_value="rl_ekf.yaml",
            description="EKF YAML inside the ekf package's config dir "
                        "(rl_ekf.yaml = new 16-state quaternion EKF; "
                        "rl_ekf_legacy.yaml = 15-state Euler for A/B)"),
        DeclareLaunchArgument(
            "ov_config", default_value="zephyrus_real",
            description="OpenVINS config dir name under ov_msckf/config/. "
                        "Defaults to 'zephyrus_real' (untracked: gazebo_sim "
                        "+ init_dyn_use:true, required for our motion-from-t0 "
                        "bag). Use 'gazebo_sim' if you only have the upstream "
                        "configs and a bag with a static start phase."),
        DeclareLaunchArgument(
            "image_topic", default_value="/cam0/image_raw"),
        DeclareLaunchArgument(
            "publish_static_world_to_map", default_value="true",
            description="Publish a static TF world↔map (Vicon's frame) so "
                        "EKF/Vicon can be compared in a common frame. The "
                        "transform is identity by default; the eval notebook "
                        "computes the correct rotation via Umeyama and prints "
                        "values to hardcode here for the next test."),
    ]

    use_sim_time = LaunchConfiguration("use_sim_time")
    ekf_config_file = LaunchConfiguration("ekf_config_file")
    ov_config = LaunchConfiguration("ov_config")
    image_topic = LaunchConfiguration("image_topic")
    publish_w2m = LaunchConfiguration("publish_static_world_to_map")

    # ------------------------------------------------------------------
    # 1) Static TF world ↔ map (Vicon root frame)
    #    Default is identity — Vicon poses are stamped in 'map'. EKF
    #    publishes in 'world'. Without this static publisher, RViz can't
    #    show both at once. The Umeyama alignment in eval_replay.py
    #    discovers the correct rotation; replace the values below for
    #    future tests.
    # ------------------------------------------------------------------
    static_tf_world_map = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_map_static_tf",
        arguments=["0", "0", "0", "0", "0", "0", "world", "map"],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(publish_w2m),
    )

    # OpenVINS publishes /ov_msckf/odomimu with header.frame_id="global" and
    # child_frame_id="imu". The EKF (rl_ekf.yaml) is configured with
    # world_frame: world and base_link_frame: base_link, so it tries to
    # TF-transform incoming odometry from global→world and twist from
    # imu→base_link. Without these static identities, every VIO message is
    # rejected by robot_localization (pre-fix this gave count=0 on /ekf/odometry).
    static_tf_world_global = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_global_static_tf",
        arguments=["0", "0", "0", "0", "0", "0", "world", "global"],
        parameters=[{"use_sim_time": use_sim_time}],
    )
    static_tf_baselink_imu = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_imu_static_tf",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "imu"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # ------------------------------------------------------------------
    # 2) Gate detection — uses its own hardcoded intrinsics (per package
    #    state in HEAD; see GATE_DETECTION_FINDINGS.md)
    # ------------------------------------------------------------------
    gate_detection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("gate_detection"), "launch", "gate_detection.launch.py"]
            )
        ),
        launch_arguments={
            "camera_topic": image_topic,
        }.items(),
    )

    # ------------------------------------------------------------------
    # 3) EKF (robot_localization) — selectable config for new vs legacy A/B
    # ------------------------------------------------------------------
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("ekf"), "config", ekf_config_file]
            ),
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("cmd_vel", "/control/command"),
            ("odometry/filtered", "/ekf/odometry"),
        ],
    )

    # ------------------------------------------------------------------
    # 4) OpenVINS MSCKF subscribe mode, pointed at zephyrus_real config
    # ------------------------------------------------------------------
    ov_config_path = PathJoinSubstitution(
        [FindPackageShare("ov_msckf"), "config", ov_config, "estimator_config.yaml"]
    )
    vio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ov_msckf"), "launch", "subscribe.launch.py"]
            )
        ),
        launch_arguments={
            "config": ov_config,
            "config_path": ov_config_path,
            "use_sim_time": use_sim_time,
            "use_stereo": "false",
            "max_cameras": "1",
        }.items(),
    )

    return LaunchDescription([
        *args,
        static_tf_world_map,
        static_tf_world_global,
        static_tf_baselink_imu,
        gate_detection_launch,
        ekf_node,
        vio_launch,
    ])
