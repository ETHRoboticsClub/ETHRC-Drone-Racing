#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray
import numpy as np

class TiltToThrottleNode(Node):
    def __init__(self):
        super().__init__('test_tilt_to_throttle')
        
        # Parameters
        self.declare_parameter('imu_topic', '/imu0')
        self.declare_parameter('cmd_topic', '/drone_attitude_cmd')
        self.declare_parameter('max_tilt_rad', 0.5) # ~30 degrees
        self.declare_parameter('base_throttle', 0.1)
        self.declare_parameter('max_throttle_gain', 0.4)
        
        imu_topic = self.get_parameter('imu_topic').value
        cmd_topic = self.get_parameter('cmd_topic').value
        self.max_tilt_rad = self.get_parameter('max_tilt_rad').value
        self.base_throttle = self.get_parameter('base_throttle').value
        self.max_gain = self.get_parameter('max_throttle_gain').value
        
        # Publisher
        self.cmd_pub = self.create_publisher(Float64MultiArray, cmd_topic, 10)
        
        # Subscriber
        self.imu_sub = self.create_subscription(Imu, imu_topic, self.imu_callback, 10)
        
        self.get_logger().info(f'Tilt-to-Throttle Verification Node Started.')
        self.get_logger().info(f'Listening on {imu_topic}, publishing to {cmd_topic}')

    def imu_callback(self, msg):
        # Extremely simplified tilt estimation (normalized gravity vector)
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        
        # Total acceleration
        norm = np.sqrt(ax**2 + ay**2 + az**2)
        if norm < 0.1: return
        
        # Normalized gravity components
        nx, ny, nz = ax/norm, ay/norm, az/norm
        
        # Tilt angle (deviation from Z-axis)
        tilt_rad = np.arccos(np.clip(nz, -1.0, 1.0))
        
        # Throttle mapping: more tilt = more throttle
        # Scaled from 0 to max_tilt_rad
        tilt_factor = np.clip(tilt_rad / self.max_tilt_rad, 0.0, 1.0)
        
        throttle = self.base_throttle + tilt_factor * self.max_gain
        
        # Publish attitude command [Roll_Rate, Pitch_Rate, Yaw_Rate, Throttle]
        # For this test, stay level (zero rates) but increase throttle
        cmd_msg = Float64MultiArray()
        cmd_msg.data = [0.0, 0.0, 0.0, float(throttle)]
        self.cmd_pub.publish(cmd_msg)
        
        if self.get_clock().now().nanoseconds % 1000000000 < 50000000: # Log at ~1Hz
            self.get_logger().info(f'Tilt: {np.degrees(tilt_rad):.1f} deg | Calculated Throttle: {throttle:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = TiltToThrottleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
