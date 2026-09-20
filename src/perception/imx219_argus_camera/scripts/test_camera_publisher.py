#!/usr/bin/env python3
"""
Synthetic camera publisher for testing the perception pipeline on x86.

Publishes a 1280x720 MONO8 test pattern to /cam0/image_raw and a matching
CameraInfo to /cam0/camera_info at a configurable frame rate.

Usage:
    ros2 run imx219_argus_camera test_camera_publisher.py --ros-args -p fps:=30
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class TestCameraPublisher(Node):
    def __init__(self):
        super().__init__("test_camera_publisher")

        self.declare_parameter("fps", 30)
        self.declare_parameter("width", 1280)
        self.declare_parameter("height", 720)
        self.declare_parameter("frame_id", "cam0")

        fps = float(self.get_parameter("fps").value)
        self.width = self.get_parameter("width").value
        self.height = self.get_parameter("height").value
        self.frame_id = self.get_parameter("frame_id").value

        self.image_pub = self.create_publisher(Image, "/cam0/image_raw", 10)
        self.info_pub = self.create_publisher(CameraInfo, "/cam0/camera_info", 10)

        # Pre-allocate gradient frame and reuse across publishes
        gradient = np.linspace(0, 200, self.width, dtype=np.uint8)
        self._base_frame = np.tile(gradient, (self.height, 1))

        # Pre-build the static CameraInfo
        self._info_msg = CameraInfo()
        self._info_msg.header.frame_id = self.frame_id
        self._info_msg.width = self.width
        self._info_msg.height = self.height
        self._info_msg.distortion_model = "plumb_bob"
        self._info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        self._info_msg.k = [500.0, 0.0, 640.0, 0.0, 500.0, 360.0, 0.0, 0.0, 1.0]
        self._info_msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self._info_msg.p = [
            500.0, 0.0, 640.0, 0.0,
            0.0, 500.0, 360.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]

        period = 1.0 / fps
        self.timer = self.create_timer(period, self.publish_frame)
        self.frame_count = 0

        self.get_logger().info(
            f"Test camera publisher: {self.width}x{self.height} @ {fps} Hz"
        )

    def publish_frame(self):
        stamp = self.get_clock().now().to_msg()

        img_msg = Image()
        img_msg.header.stamp = stamp
        img_msg.header.frame_id = self.frame_id
        img_msg.height = self.height
        img_msg.width = self.width
        img_msg.encoding = "mono8"
        img_msg.is_bigendian = False
        img_msg.step = self.width

        # Moving bright bar over pre-allocated gradient
        frame = self._base_frame.copy()
        bar_x = (self.frame_count * 4) % self.width
        bar_end = min(bar_x + 4, self.width)
        frame[:, bar_x:bar_end] = 255
        img_msg.data = frame.tobytes()

        self._info_msg.header.stamp = stamp
        self.image_pub.publish(img_msg)
        self.info_pub.publish(self._info_msg)
        self.frame_count += 1


def main():
    rclpy.init()
    node = TestCameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
