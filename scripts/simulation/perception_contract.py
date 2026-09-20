"""Static contract checks for the first Gazebo SITL perception ingress slice.

The checks intentionally validate only configuration that is already
authoritative.  They do not claim camera calibration or add a ground-truth
fallback to perception/control.
"""

from __future__ import annotations

from pathlib import Path
from typing import Iterable
import xml.etree.ElementTree as ET

import yaml


EXPECTED_BRIDGES = {
    "simulation_clock": ("/clock", "rosgraph_msgs/msg/Clock", "gz.msgs.Clock", "/clock"),
    "camera_image": ("/drone/camera/image", "sensor_msgs/msg/Image", "gz.msgs.Image", "/cam0/image_raw"),
    "camera_info": ("/drone/camera/camera_info", "sensor_msgs/msg/CameraInfo", "gz.msgs.CameraInfo", "/cam0/camera_info"),
    "camera_imu": ("/drone/imu", "sensor_msgs/msg/Imu", "gz.msgs.IMU", "/imu0"),
}


def _errors_for_bridge_config(path: Path) -> list[str]:
    data = yaml.safe_load(path.read_text())
    bridges = {bridge.get("name"): bridge for bridge in data.get("bridges", [])}
    errors: list[str] = []
    for name, (gz_topic, ros_type, gz_type, remap_to) in EXPECTED_BRIDGES.items():
        bridge = bridges.get(name)
        if bridge is None:
            errors.append(f"missing bridge '{name}'")
            continue
        actual = (bridge.get("gz_topic"), bridge.get("ros2_type"), bridge.get("gz_type"), bridge.get("remap_to"))
        expected = (gz_topic, ros_type, gz_type, remap_to)
        if actual != expected:
            errors.append(f"bridge '{name}' is {actual!r}, expected {expected!r}")
        if bridge.get("direction", "GZ_TO_ROS") != "GZ_TO_ROS":
            errors.append(f"bridge '{name}' must be GZ_TO_ROS")
        if bridge.get("enabled", True) is not True:
            errors.append(f"bridge '{name}' must be enabled")
    return errors


def _errors_for_model(path: Path) -> list[str]:
    root = ET.parse(path).getroot()
    sensors = {sensor.get("name"): sensor for sensor in root.findall(".//sensor")}
    errors: list[str] = []
    imu = sensors.get("imu_sensor")
    camera = sensors.get("front_camera")
    if imu is None:
        errors.append("model is missing imu_sensor")
    else:
        if imu.findtext("topic") != "/drone/imu":
            errors.append("imu_sensor must publish /drone/imu")
        if imu.findtext("update_rate") != "250":
            errors.append("imu_sensor update rate must remain 250 Hz")
    if camera is None:
        errors.append("model is missing front_camera")
    else:
        if camera.findtext("topic") != "/drone/camera/image":
            errors.append("front_camera must publish /drone/camera/image")
        if camera.findtext("update_rate") != "30":
            errors.append("front_camera update rate must remain 30 Hz")
    return errors


def validate_contract(bridge_config: Path, model_sdf: Path) -> list[str]:
    """Return all static ingress contract violations, or an empty list."""
    return _errors_for_bridge_config(bridge_config) + _errors_for_model(model_sdf)


def format_errors(errors: Iterable[str]) -> str:
    return "\n".join(f"- {error}" for error in errors)
