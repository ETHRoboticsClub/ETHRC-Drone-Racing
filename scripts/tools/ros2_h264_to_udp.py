#!/usr/bin/env python3
import socket
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

class ROS2H264ToUDP(Node):
    def __init__(self, port=6000):
        super().__init__('ros2_h264_to_udp')
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.target = ('127.0.0.1', self.port)
        
        self.get_logger().info(f"Forwarding /cam0/image_h264 packets to UDP 127.0.0.1:{self.port}...")
        self.get_logger().info(f"To decode and view, run:")
        self.get_logger().info(f"  gst-launch-1.0 udpsrc port={self.port} ! h264parse ! avdec_h264 ! videoconvert ! autovideosink")

        self.subscription = self.create_subscription(
            CompressedImage,
            '/cam0/image_h264',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        # H.264 UDP MTU fragment warning:
        # Standard UDP sockets can handle up to 65KB packets.
        # 720p H.264 P-frames are usually 5-20KB, well within bounds.
        # Keyframes (I-frames) can exceed 65KB if bitrate is extremely high,
        # but at standard drone rates, they fit within UDP buffer limits.
        try:
            self.sock.sendto(bytes(msg.data), self.target)
        except Exception as e:
            self.get_logger().warn(f"Failed to forward packet: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ROS2H264ToUDP()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sock.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
