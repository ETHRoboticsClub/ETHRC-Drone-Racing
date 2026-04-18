#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class SinThrustPublisherNode(Node):
    """Publish [0, 0, 0, sin(2*pi*t)] attitude rate commands at a fixed rate."""

    def __init__(self) -> None:
        super().__init__("sin_thrust_publisher")

        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("frequency", 1.0)
        self.declare_parameter("amplitude", 0.5)
        self.declare_parameter("offset", 0.5)

        topic = "/offboard/attitude_rate_cmd"
        rate_hz = self.get_parameter("rate_hz").get_parameter_value().double_value
        self._freq = self.get_parameter("frequency").get_parameter_value().double_value
        self._amp = self.get_parameter("amplitude").get_parameter_value().double_value
        self._offset = self.get_parameter("offset").get_parameter_value().double_value

        self._pub = self.create_publisher(Float32MultiArray, topic, 10)
        self._t0 = self.get_clock().now()
        self.create_timer(1.0 / rate_hz, self._tick)

        self.get_logger().info(
            f"Publishing sin thrust on '{topic}' at {rate_hz} Hz "
            f"(freq={self._freq} Hz, amp={self._amp}, offset={self._offset})"
        )

    def _tick(self) -> None:
        t = (self.get_clock().now() - self._t0).nanoseconds * 1e-9
        thrust = self._offset + self._amp * math.sin(2.0 * math.pi * self._freq * t)
        thrust = max(0.0, min(1.0, thrust))

        msg = Float32MultiArray()
        msg.data = [0.0, 0.0, 0.0, thrust]
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SinThrustPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
