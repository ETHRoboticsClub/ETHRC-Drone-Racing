#!/usr/bin/env python3
"""
Subscribe to the drone's camera ROS 2 topic and display the video feed.

The argus_camera_node publishes sensor_msgs/Image on /cam0/image_raw (mono8)
and optionally /cam0/image_rgb (rgb8).  This script subscribes, converts
each frame, and shows it in an OpenCV window.

Usage:
    ros2 run groundstation receive_video_stream            # default topic
    python3 receive_video_stream.py                        # standalone
    python3 receive_video_stream.py --topic /cam0/image_rgb
    python3 receive_video_stream.py --save recording.mp4
"""

import argparse
import sys

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image


class VideoReceiver(Node):
    def __init__(self, topic: str, save_path: str | None = None):
        super().__init__("video_receiver")

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.subscription = self.create_subscription(
            Image, topic, self._image_cb, qos
        )
        self.get_logger().info(f"Subscribed to {topic}  —  press 'q' or ESC to quit")

        self._save_path = save_path
        self._writer: cv2.VideoWriter | None = None
        self._frame_count = 0

    # ── callback ──────────────────────────────────────────────────────
    def _image_cb(self, msg: Image):
        # Convert ROS Image to numpy array
        if msg.encoding in ("mono8",):
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width
            )
            display = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        elif msg.encoding in ("rgb8",):
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 3
            )
            display = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        elif msg.encoding in ("bgr8",):
            display = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 3
            )
        else:
            self.get_logger().warn(
                f"Unsupported encoding '{msg.encoding}', skipping frame",
                throttle_duration_sec=5.0,
            )
            return

        # Lazily init video writer once we know the frame size
        if self._save_path and self._writer is None:
            h, w = display.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            self._writer = cv2.VideoWriter(self._save_path, fourcc, 60, (w, h))
            self.get_logger().info(f"Saving to {self._save_path} ({w}x{h})")

        if self._writer is not None:
            self._writer.write(display)

        self._frame_count += 1
        cv2.imshow("Drone Camera Stream", display)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            self.get_logger().info(
                f"Quit requested after {self._frame_count} frames."
            )
            raise SystemExit

    # ── cleanup ───────────────────────────────────────────────────────
    def destroy_node(self):
        if self._writer is not None:
            self._writer.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser(
        description="Display the drone camera feed from a ROS 2 image topic."
    )
    parser.add_argument(
        "--topic", type=str, default="/cam0/image_raw",
        help="ROS 2 image topic to subscribe to (default: /cam0/image_raw)",
    )
    parser.add_argument(
        "--save", type=str, default=None, metavar="FILE",
        help="Optional path to save the received video (e.g. recording.mp4)",
    )
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = VideoReceiver(args.topic, args.save)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
