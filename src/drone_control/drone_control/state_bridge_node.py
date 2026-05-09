#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray
import numpy as np
import time

class DirectStateBridgeNode(Node):
    """
    Bypasses EKF by fusing Vicon Pose and IMU data directly.
    Provides a 13-element state array for the RL controller.
    """
    def __init__(self):
        super().__init__('state_estimator_bridge')
        
        self.declare_parameter('vicon_topic', '/vicon/zephyrus/zephyrus')
        self.declare_parameter('imu_topic', '/imu0')
        self.declare_parameter('state_topic', 'drone_state_sim')
        self.declare_parameter('publish_rate', 100.0) # Hz
        
        vicon_topic = self.get_parameter('vicon_topic').value
        imu_topic = self.get_parameter('imu_topic').value
        state_topic = self.get_parameter('state_topic').value
        publish_rate = self.get_parameter('publish_rate').value
        
        # Latest data buffers
        self.latest_pos = np.zeros(3)
        self.latest_quat = np.array([0.0, 0.0, 0.0, 1.0])
        self.latest_vel = np.zeros(3)
        self.latest_omega = np.zeros(3)
        
        # Velocity estimation state
        self.prev_pos = None
        self.prev_time = None
        
        # Subscriptions
        self.vicon_sub = self.create_subscription(
            PoseStamped, vicon_topic, self.vicon_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, imu_topic, self.imu_callback, 10)
            
        # Publisher
        self.publisher = self.create_publisher(Float64MultiArray, state_topic, 10)
        
        # Timer for fixed-rate output
        self.timer = self.create_timer(1.0/publish_rate, self.timer_callback)
        
        self.get_logger().info(f'Direct State Bridge started: Vicon({vicon_topic}) + IMU({imu_topic}) -> {state_topic} @ {publish_rate}Hz')

    def vicon_callback(self, msg: PoseStamped):
        curr_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        curr_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        # Simple velocity differentiation
        if self.prev_pos is not None and self.prev_time is not None:
            dt = curr_time - self.prev_time
            if dt > 0:
                raw_vel = (curr_pos - self.prev_pos) / dt
                # Low-pass filter (alpha=0.5) to handle noisy Vicon derivatives
                self.latest_vel = 0.5 * self.latest_vel + 0.5 * raw_vel
        
        self.latest_pos = curr_pos
        self.latest_quat = np.array([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ])
        
        self.prev_pos = curr_pos
        self.prev_time = curr_time

    def imu_callback(self, msg: Imu):
        # Extract angular velocity directly from high-rate IMU
        self.latest_omega = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])

    def timer_callback(self):
        # Construct 13-element state: [p(3), v(3), q(4), w(3)]
        # Note: RL Controller expects [pos, vel, quat, omega] or [pos, quat, vel, omega]?
        # Earlier bridge used: [p(3), v(3), q(4), w(3)]
        
        data = [
            float(self.latest_pos[0]), float(self.latest_pos[1]), float(self.latest_pos[2]),
            float(self.latest_vel[0]), float(self.latest_vel[1]), float(self.latest_vel[2]),
            float(self.latest_quat[0]), float(self.latest_quat[1]), float(self.latest_quat[2]), float(self.latest_quat[3]),
            float(self.latest_omega[0]), float(self.latest_omega[1]), float(self.latest_omega[2])
        ]
        
        # Sanity check for NaNs (from Vicon dropout)
        if np.isnan(data).any():
            return

        state_msg = Float64MultiArray()
        state_msg.data = data
        self.publisher.publish(state_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DirectStateBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
