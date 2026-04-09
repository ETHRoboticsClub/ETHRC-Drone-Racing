from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    # Camera bridge (Gazebo ↔ ROS 2)
    camera_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("gazebo_ros_bridges"), "launch", "bridge.launch.py"]
            )
        )
    )

    # Vision (Gate Detection - WIP)
    gate_detection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("gate_detection"), "launch", "gate_detection.launch.py"]
            )
        )
    )

    # EKF (robot_localization)
    ekf_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ekf"), "launch", "rl_ekf.launch.xml"]
            )
        )
    )

    # VIO (OpenVINS MSCKF subscribe mode); TODO: Run this correctly!
    ov_config_path = PathJoinSubstitution(
        [FindPackageShare("ov_msckf"), "config", "gazebo_sim", "estimator_config.yaml"]
    )
    vio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ov_msckf"), "launch", "subscribe.launch.py"]
            )
        ),
        launch_arguments={
            "config": "gazebo_sim",
            "config_path": ov_config_path,
            "use_stereo": "false",
            "max_cameras": "1",
        }.items(),
    )

    return LaunchDescription(
        [
            camera_bridge_launch,
            # aruco_launch, # Archived
            gate_detection_launch,
            ekf_launch,
            vio_launch,
        ]
    )
