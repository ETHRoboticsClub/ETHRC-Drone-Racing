#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageViewer(Node):
    def __init__(self):
        super().__init__('image_viewer')
        self.bridge = CvBridge()

        # Store the latest images
        self.distorted_image = None
        self.undistorted_image = None
        self.aruco_debug_image = None

        # Subscribe to distorted image
        self.subscription_distorted = self.create_subscription(
            Image,
            'drone/camera/image',
            self.distorted_callback,
            10)

        # Subscribe to undistorted/calibrated image
        self.subscription_undistorted = self.create_subscription(
            Image,
            'drone/camera/image_calib',
            self.undistorted_callback,
            10)

        # Subscribe to ArUco debug image
        self.subscription_aruco_debug = self.create_subscription(
            Image,
            '/aruco/debug_image',
            self.aruco_debug_callback,
            10)

        self.get_logger().info("Image viewer started - showing distorted, undistorted, and ArUco debug images")

    def distorted_callback(self, msg):
        try:
            self.distorted_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imshow("Distorted Image (Original)", self.distorted_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error in distorted callback: {e}")

    def undistorted_callback(self, msg):
        try:
            self.undistorted_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imshow("Undistorted Image (Calibrated)", self.undistorted_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error in undistorted callback: {e}")

    def aruco_debug_callback(self, msg):
        try:
            self.aruco_debug_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imshow("ArUco Detection Debug", self.aruco_debug_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error in ArUco debug callback: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageViewer()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
