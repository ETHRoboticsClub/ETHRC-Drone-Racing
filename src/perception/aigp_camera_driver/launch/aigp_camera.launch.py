import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    pkg_share = get_package_share_directory("aigp_camera_driver")
    default_config = os.path.join(pkg_share, "config", "camera_params.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config",
            default_value=default_config,
            description="Path to the vision driver parameter YAML",
        ),
        ComposableNodeContainer(
            name="aigp_camera_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container",
            output="screen",
            composable_node_descriptions=[
                ComposableNode(
                    package="aigp_camera_driver",
                    plugin="aigp_camera_driver::VisionDriverNode",
                    name="aigp_vision_driver_node",
                    parameters=[LaunchConfiguration("config")],
                    remappings=[
                        ("image_raw", "/cam0/image_raw"),
                        ("image_rgb", "/cam0/image_rgb"),
                        ("camera_info", "/cam0/camera_info"),
                    ],
                    extra_arguments=[{"use_intra_process_comms": True}],
                ),
            ],
        ),
    ])
