#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R
import numpy as np
import time

class DirectStateBridgeNode(Node):
    """
    Bypasses EKF by fusing Pose (Vicon or Gazebo Odometry) and IMU data directly.
    Provides a 13-element state array for the RL controller.
    """
    def __init__(self):
        super().__init__('state_estimator_bridge')
        
        self.declare_parameter('pose_source', 'vicon') # 'vicon' or 'odometry'
        self.declare_parameter('vicon_topic', '/vicon/zephyrus/zephyrus')
        self.declare_parameter('odometry_topic', '/odometry')
        self.declare_parameter('imu_topic', '/imu0')
        self.declare_parameter('state_topic', 'drone_state_sim')
        self.declare_parameter('publish_rate', 100.0) # Hz
        
        self.pose_source = self.get_parameter('pose_source').value
        vicon_topic = self.get_parameter('vicon_topic').value
        odometry_topic = self.get_parameter('odometry_topic').value
        imu_topic = self.get_parameter('imu_topic').value
        state_topic = self.get_parameter('state_topic').value
        publish_rate = self.get_parameter('publish_rate').value
        
        # Latest data buffers
        self.latest_pos = np.zeros(3)
        self.latest_quat = np.array([0.0, 0.0, 0.0, 1.0])
        self.latest_vel = np.zeros(3)
        self.latest_omega = np.zeros(3)
        self.has_vicon_data = False # Reused as has_pose_data
        self.has_imu_data = False
        
        # Velocity estimation state (only used for Vicon numerical differentiation)
        self.prev_pos = None
        self.prev_time = None
        
        # Subscriptions
        if self.pose_source == 'vicon':
            self.vicon_sub = self.create_subscription(
                PoseStamped, vicon_topic, self.vicon_callback, 10)
            self.get_logger().info(f'Subscribed to Vicon: {vicon_topic}')
        elif self.pose_source == 'odometry':
            self.odom_sub = self.create_subscription(
                Odometry, odometry_topic, self.odom_callback, 10)
            self.get_logger().info(f'Subscribed to Odometry: {odometry_topic}')
        else:
            self.get_logger().error(f'Invalid pose_source: {self.pose_source}')
        
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        self.imu_sub = self.create_subscription(
            Imu, imu_topic, self.imu_callback,
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10))
            
        # Publisher
        self.publisher = self.create_publisher(Float64MultiArray, state_topic, 10)
        
        # Timer for fixed-rate output
        self.timer = self.create_timer(1.0/publish_rate, self.timer_callback)
        
        self.get_logger().info(f'Direct State Bridge started: Source({self.pose_source}) + IMU({imu_topic}) -> {state_topic} @ {publish_rate}Hz')

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
        self.has_vicon_data = True

    def odom_callback(self, msg: Odometry):
        self.latest_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        
        self.latest_quat = np.array([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])
        
        # Linear velocity extraction
        vel_msg = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ])
        
        # Rotate linear velocity to world frame if it's in body frame
        if msg.child_frame_id and msg.child_frame_id != msg.header.frame_id:
            r = R.from_quat(self.latest_quat)
            self.latest_vel = r.apply(vel_msg)
        else:
            self.latest_vel = vel_msg
            
        self.has_vicon_data = True

    def imu_callback(self, msg: Imu):
        # Extract angular velocity directly from high-rate IMU
        self.latest_omega = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
        self.has_imu_data = True

    def timer_callback(self):
        if not self.has_vicon_data or not self.has_imu_data:
            self.get_logger().warn(
                f"Waiting for sensor data: Vicon={self.has_vicon_data}, IMU={self.has_imu_data}...",
                throttle_duration_sec=2.0
            )
            return

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
