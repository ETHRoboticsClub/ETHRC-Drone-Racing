"""Replay-mode pipeline that substitutes Vicon for VIO at the EKF's
production input topic.

Why this exists
---------------
The 16-state EKF takes ``nav_msgs/Odometry`` from OpenVINS on
``/ov_msckf/odomimu`` in flight (per ``rl_ekf.yaml``). That same code
path is what ``ekf/src/nodes/mock_publisher.cpp`` exercised during the
refactor, and it survived those tests.

We can't use real OpenVINS on bag 20260416_175153 because IMU↔camera
calibration is unresolved (kilometre drift). But we can publish a
*synthesised* ``/ov_msckf/odomimu`` from Vicon — high-quality pose at
100 Hz with zero twist + loose twist covariance. That hits exactly the
``odom0`` code path the EKF is built for.

This launch deliberately does NOT run:
    - OpenVINS    (would diverge; we're standing in for it)
    - gate_detection (broken/WIP and we don't need pose0 corrections
                     for this validation)
    - camera_info_republisher (no camera consumers downstream)

It runs:
    - vicon_to_odometry.py   → publishes /ov_msckf/odomimu
    - 2 static TFs           → world↔map (Vicon's frame), base_link↔imu
    - ekf_filter_node        → with rl_ekf.yaml UNCHANGED (production)

Run (new EKF):
    ros2 launch launch/perception_replay_vio_subst.launch.py
    # in B:
    ros2 bag record -o out/replay_175153_subst_new \\
        /ekf/odometry /tf /tf_static /ov_msckf/odomimu
    # in C:
    ros2 bag play /home/drone_dev/data_20260416/perception/20260416_175153 \\
        --clock

Run (legacy EKF for A/B):
    EKF_CONFIG_FILE=rl_ekf_legacy.yaml \\
    LAUNCH_FILE=launch/perception_replay_vio_subst.launch.py \\
        ./scripts/perception/replay/run_legacy_ekf_replay.sh
"""
from __future__ import annotations

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    this_file = os.path.realpath(__file__)
    ws_root = os.path.realpath(os.path.join(os.path.dirname(this_file), ".."))
    default_adapter_path = os.path.join(
        ws_root, "scripts", "perception", "replay", "vicon_to_odometry.py"
    )

    args = [
        DeclareLaunchArgument(
            "use_sim_time", default_value="true",
            description="Use /clock from `ros2 bag play --clock`"),
        DeclareLaunchArgument(
            "ekf_config_file", default_value="rl_ekf.yaml",
            description="EKF YAML inside the ekf package's config dir "
                        "(rl_ekf.yaml = new 16-state, "
                        "rl_ekf_legacy.yaml = legacy 15-state for A/B)"),
        DeclareLaunchArgument(
            "vicon_in_topic", default_value="/vicon/zephyrus/zephyrus"),
        DeclareLaunchArgument(
            "vio_out_topic", default_value="/ov_msckf/odomimu"),
        DeclareLaunchArgument(
            "adapter_path", default_value=default_adapter_path),
    ]

    use_sim_time = LaunchConfiguration("use_sim_time")
    ekf_config_file = LaunchConfiguration("ekf_config_file")
    vicon_in = LaunchConfiguration("vicon_in_topic")
    vio_out = LaunchConfiguration("vio_out_topic")
    adapter_path = LaunchConfiguration("adapter_path")

    # Vicon → /ov_msckf/odomimu adapter
    vicon_adapter = ExecuteProcess(
        cmd=[
            "python3", adapter_path,
            "--input", vicon_in,
            "--output", vio_out,
            "--frame-id", "world",
            "--child-frame-id", "base_link",
            "--ros-args", "-p", ["use_sim_time:=", use_sim_time],
        ],
        output="screen",
        name="vicon_to_odometry",
    )

    # Static TFs:
    #  - world ↔ map: the bag's Vicon publishes in 'map'. The adapter
    #    re-stamps its output Odometry as frame_id='world' so the EKF
    #    gets data already in its world_frame; this TF exists for any
    #    other consumer that still expects 'map' (e.g. RViz overlays
    #    of the original Vicon stream).
    #  - base_link ↔ imu_link: harmless identity, kept for symmetry
    #    with the production launch.
    static_tf_world_map = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_map_static_tf",
        arguments=["0", "0", "0", "0", "0", "0", "world", "map"],
        parameters=[{"use_sim_time": use_sim_time}],
    )
    static_tf_baselink_imu = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_imu_link_static_tf",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "imu_link"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # EKF — production config unchanged
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

    return LaunchDescription([
        *args,
        vicon_adapter,
        static_tf_world_map,
        static_tf_baselink_imu,
        ekf_node,
    ])
