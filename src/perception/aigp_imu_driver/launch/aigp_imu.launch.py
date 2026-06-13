import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    pkg_share = get_package_share_directory("aigp_imu_driver")
    default_config = os.path.join(pkg_share, "config", "imu_params.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config",
            default_value=default_config,
            description="Path to the IMU driver parameter YAML",
        ),
        ComposableNodeContainer(
            name="aigp_imu_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container",
            output="screen",
            composable_node_descriptions=[
                ComposableNode(
                    package="aigp_imu_driver",
                    plugin="aigp_imu_driver::ImuBridgeNode",
                    name="aigp_imu_bridge_node",
                    parameters=[LaunchConfiguration("config")],
                    extra_arguments=[{"use_intra_process_comms": True}],
                ),
            ],
        ),
    ])
