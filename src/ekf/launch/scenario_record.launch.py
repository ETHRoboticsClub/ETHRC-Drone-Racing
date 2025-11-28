#!/usr/bin/env python3

from multiprocessing import context
import shlex
from pathlib import Path

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

CSV_HEADER = (
    "t_sec,t_nsec,_,_,x,y,z,xq,yq,zq,w,pose_cov_0,pose_cov_1,pose_cov_2,pose_cov_3,pose_cov_4,pose_cov_5,"
    "pose_cov_6,pose_cov_7,pose_cov_8,pose_cov_9,pose_cov_10,pose_cov_11,pose_cov_12,pose_cov_13,pose_cov_14,"
    "pose_cov_15,pose_cov_16,pose_cov_17,pose_cov_18,pose_cov_19,pose_cov_20,pose_cov_21,pose_cov_22,pose_cov_23,"
    "pose_cov_24,pose_cov_25,pose_cov_26,pose_cov_27,pose_cov_28,pose_cov_29,pose_cov_30,pose_cov_31,pose_cov_32,"
    "pose_cov_33,pose_cov_34,pose_cov_35,vx,vy,vz,wx,wy,wz,twist_cov_0,twist_cov_1,twist_cov_2,twist_cov_3,"
    "twist_cov_4,twist_cov_5,twist_cov_6,twist_cov_7,twist_cov_8,twist_cov_9,twist_cov_10,twist_cov_11,twist_cov_12,"
    "twist_cov_13,twist_cov_14,twist_cov_15,twist_cov_16,twist_cov_17,twist_cov_18,twist_cov_19,twist_cov_20,"
    "twist_cov_21,twist_cov_22,twist_cov_23,twist_cov_24,twist_cov_25,twist_cov_26,twist_cov_27,twist_cov_28,"
    "twist_cov_29,twist_cov_30,twist_cov_31,twist_cov_32,twist_cov_33,twist_cov_34,twist_cov_35"
)

ACCEL_CSV_HEADER = (
    "t_sec,t_nsec,_,ax,ay,az,wx,wy,wz,"
    "accel_cov_0,accel_cov_1,accel_cov_2,accel_cov_3,accel_cov_4,accel_cov_5,"
    "accel_cov_6,accel_cov_7,accel_cov_8,accel_cov_9,accel_cov_10,accel_cov_11,"
    "accel_cov_12,accel_cov_13,accel_cov_14,accel_cov_15,accel_cov_16,accel_cov_17,"
    "accel_cov_18,accel_cov_19,accel_cov_20,accel_cov_21,accel_cov_22,accel_cov_23,"
    "accel_cov_24,accel_cov_25,accel_cov_26,accel_cov_27,accel_cov_28,accel_cov_29,"
    "accel_cov_30,accel_cov_31,accel_cov_32,accel_cov_33,accel_cov_34,accel_cov_35"
)


def _default_rl_ekf_path():
    """
    Resolve the rl_ekf.launch.xml shipped with this package, falling back to
    the robot_localization version if needed.
    """
    try:
        return str(get_package_share_path("ekf") / "launch" / "rl_ekf.launch.xml")
    except Exception:
        try:
            return str(get_package_share_path("robot_localization") / "launch" / "rl_ekf.launch.xml")
        except Exception:
            return ""


def generate_launch_description():
    default_rl_ekf = _default_rl_ekf_path()

    scenario = LaunchConfiguration("scenario")
    bagname = LaunchConfiguration("bagname")
    duration = LaunchConfiguration("duration")
    seed = LaunchConfiguration("seed")
    vio_rate = LaunchConfiguration("vio_rate_hz")
    vision_rate = LaunchConfiguration("vision_rate_hz")
    rl_ekf_path = LaunchConfiguration("rl_ekf_path")

    def _resolve_bagdir_path(context):
        scenario_val = context.perform_substitution(scenario)
        bagname_val = context.perform_substitution(bagname)
        bagdir_name = f"{scenario_val}_{bagname_val}"
        bag_root = Path.cwd() / "tests"
        bag_root.mkdir(parents=True, exist_ok=True)
        bagdir_path = bag_root / bagdir_name
        bagdir_path.mkdir(parents=True, exist_ok=True)
        return bagdir_path

    def _prepare_csv(context):
        bagdir_path = _resolve_bagdir_path(context)

        # EKF odometry CSV
        csv_path = bagdir_path / "scenario_data.csv"
        if not csv_path.exists():
            csv_path.write_text(CSV_HEADER + "\n")

        # Acceleration CSV from /accel/filtered
        accel_csv_path = bagdir_path / "accel_data.csv"
        if not accel_csv_path.exists():
            accel_csv_path.write_text(ACCEL_CSV_HEADER + "\n")

        return bagdir_path, csv_path, accel_csv_path

    declare_args = [
        DeclareLaunchArgument("scenario", default_value=TextSubstitution(text="const_vel")),
        DeclareLaunchArgument("bagname", default_value=TextSubstitution(text="bag")),
        DeclareLaunchArgument("duration", default_value=TextSubstitution(text="20.0")),
        DeclareLaunchArgument("seed", default_value=TextSubstitution(text="42")),
        DeclareLaunchArgument("vio_rate_hz", default_value=TextSubstitution(text="100.0")),
        DeclareLaunchArgument("vision_rate_hz", default_value=TextSubstitution(text="60.0")),
        DeclareLaunchArgument("rl_ekf_path", default_value=TextSubstitution(text=default_rl_ekf)),
    ]

    mock = Node(
        package="ekf",
        executable="mock_publisher",
        name="mock_publisher",
        output="screen",
        parameters=[
            {
                "scenario": scenario,
                "duration_sec": ParameterValue(duration, value_type=float),
                "rng_seed": ParameterValue(seed, value_type=int),
                "vio_rate_hz": ParameterValue(vio_rate, value_type=float),
                "vision_rate_hz": ParameterValue(vision_rate, value_type=float),
            }
        ],
    )

    rl_ekf = IncludeLaunchDescription(XMLLaunchDescriptionSource([rl_ekf_path]))

    def _launch_recorder(context):
        bagdir_path = _resolve_bagdir_path(context)
        rosbag_path = bagdir_path / "rosbag"
        return [
            LogInfo(msg=f"Recording rosbag to {rosbag_path}"),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "bag",
                    "record",
                    "-o",
                    str(rosbag_path),
                    "/vio/odometry",
                    "/vision/pose",
                    "/odometry/filtered",
                    "/tf",
                    "/tf_static",
                    "/control/command"
                ],
                output="screen",
            ),
        ]

    def _start_csv_capture(context):
        _, csv_path, _ = _prepare_csv(context)
        csv_target = shlex.quote(str(csv_path))
        record_cmd = (
            f"echo 'Streaming /ekf/odometry to CSV at {csv_path}' >&2; "
            f"exec ros2 topic echo /ekf/odometry --csv >> {csv_target}"
        )
        return [
            LogInfo(msg=f"Streaming /ekf/odometry to CSV at {csv_path}"),
            ExecuteProcess(
                cmd=["bash", "-lc", record_cmd],
                output="screen",
            ),
        ]
    
    def _start_accel_csv_capture(context):
        _, _, accel_csv_path = _prepare_csv(context)
        accel_target = shlex.quote(str(accel_csv_path))
        record_cmd = (
            f"echo 'Streaming /accel/filtered to CSV at {accel_csv_path}' >&2; "
            f"exec ros2 topic echo /accel/filtered --csv >> {accel_target}"
        )
        return [
            LogInfo(msg=f"Streaming /accel/filtered to CSV at {accel_csv_path}"),
            ExecuteProcess(
                cmd=["bash", "-lc", record_cmd],
                output="screen",
            ),
        ]

    recorder = OpaqueFunction(function=_launch_recorder)
    csv_capture = OpaqueFunction(function=_start_csv_capture)
    accel_csv_capture = OpaqueFunction(function=_start_accel_csv_capture)

    stop_on_mock_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=mock,
            on_exit=[
                LogInfo(msg="Mock publisher completed; shutting down recorder and CSV capture."),
                Shutdown(),
            ],
        )
    )

    return LaunchDescription(declare_args + [rl_ekf, mock, recorder, csv_capture, accel_csv_capture, stop_on_mock_exit])
