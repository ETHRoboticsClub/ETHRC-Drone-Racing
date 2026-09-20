import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    # Possible to pass an alternative config from CLI
    config_path = LaunchConfiguration("config").perform(context)

    if not config_path:
        pkg_share = get_package_share_directory("gazebo_ros_bridges")
        config_path = os.path.join(pkg_share, "config", "gz_bridges.yaml")

    with open(config_path, "r") as f:
        cfg = yaml.safe_load(f)

    arguments = []
    remappings = []

    for bridge in cfg.get("bridges", []):
        if not bridge.get("enabled", True):
            continue

        gz_topic = bridge["gz_topic"]
        ros2_type = bridge["ros2_type"]
        gz_type = bridge["gz_type"]
        remap_to = bridge.get("remap_to")

        direction = bridge.get("direction", "GZ_TO_ROS").upper()
        direction_symbols = {
            "BIDIRECTIONAL": "@",
            "GZ_TO_ROS": "[",
            "ROS_TO_GZ": "]",
        }
        if direction not in direction_symbols:
            raise ValueError(
                f"Invalid bridge direction '{direction}' for {gz_topic}; "
                "expected BIDIRECTIONAL, GZ_TO_ROS, or ROS_TO_GZ"
            )

        # Build parameter_bridge argument:
        #   "<topic>@<ros2_type><direction><gz_type>"
        arguments.append(
            f"{gz_topic}@{ros2_type}{direction_symbols[direction]}{gz_type}"
        )

        if remap_to:
            remappings.append((gz_topic, remap_to))

    print(f"[gz_sensors_bridge] Using config: {config_path}")
    print(f"[gz_sensors_bridge] Bridging: {arguments}")
    print(f"[gz_sensors_bridge] Remappings: {remappings}")

    node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_sensors_bridge",
        output="screen",
        arguments=arguments,
        remappings=remappings,
    )

    return [node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "config",
            default_value="",
            description="Path to gz_bridges.yaml (optional)",
        ),
        OpaqueFunction(function=launch_setup),
    ])
