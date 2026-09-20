#!/usr/bin/env python3
import socket
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

class ROS2H264ToTCP(Node):
    def __init__(self, port=6000):
        super().__init__('ros2_h264_to_tcp')
        self.port = port
        self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_sock.bind(('127.0.0.1', self.port))
        self.server_sock.listen(1)
        self.server_sock.setblocking(False)
        self.client_sock = None
        
        self.get_logger().info(f"TCP server listening on 127.0.0.1:{self.port}...")
        self.get_logger().info(f"To decode and view, run:")
        self.get_logger().info(f"  gst-launch-1.0 tcpclientsrc host=127.0.0.1 port={self.port} ! h264parse ! avdec_h264 ! videoconvert ! autovideosink")

        self.subscription = self.create_subscription(
            CompressedImage,
            '/cam0/image_h264',
            self.listener_callback,
            10
        )
        
        # Timer to accept connections without blocking the ROS 2 executor
        self.create_timer(0.1, self.check_connections)

    def check_connections(self):
        if self.client_sock is None:
            try:
                self.client_sock, addr = self.server_sock.accept()
                self.get_logger().info(f"GStreamer connected from {addr}!")
                # Keep client socket blocking so sendall naturally buffers and blocks
                self.client_sock.setblocking(True)
            except BlockingIOError:
                pass

    def listener_callback(self, msg):
        if self.client_sock is not None:
            try:
                self.client_sock.sendall(bytes(msg.data))
            except (ConnectionResetError, BrokenPipeError, socket.error):
                self.get_logger().info("GStreamer client disconnected. Waiting for reconnect...")
                self.client_sock.close()
                self.client_sock = None

def main(args=None):
    rclpy.init(args=args)
    node = ROS2H264ToTCP()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.client_sock:
            node.client_sock.close()
        node.server_sock.close()
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
