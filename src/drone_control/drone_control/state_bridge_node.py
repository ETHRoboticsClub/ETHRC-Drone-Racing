#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray
import numpy as np
import socket
import time


_ENU_TO_NED = np.array([
    [0.0, 1.0, 0.0],
    [1.0, 0.0, 0.0],
    [0.0, 0.0, -1.0],
])
_FLU_TO_FRD = np.diag([1.0, -1.0, -1.0])


def _normalize_quat(q: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(q)
    if norm < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return q / norm


def _quat_to_rot(q: np.ndarray) -> np.ndarray:
    qw, qx, qy, qz = _normalize_quat(q)
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)],
        [2 * (qx * qy + qw * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qw * qx)],
        [2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx * qx + qy * qy)],
    ])


def _rot_to_quat(r: np.ndarray) -> np.ndarray:
    trace = np.trace(r)
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (r[2, 1] - r[1, 2]) / s
        qy = (r[0, 2] - r[2, 0]) / s
        qz = (r[1, 0] - r[0, 1]) / s
    else:
        i = int(np.argmax(np.diag(r)))
        if i == 0:
            s = np.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
            qw = (r[2, 1] - r[1, 2]) / s
            qx = 0.25 * s
            qy = (r[0, 1] + r[1, 0]) / s
            qz = (r[0, 2] + r[2, 0]) / s
        elif i == 1:
            s = np.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
            qw = (r[0, 2] - r[2, 0]) / s
            qx = (r[0, 1] + r[1, 0]) / s
            qy = 0.25 * s
            qz = (r[1, 2] + r[2, 1]) / s
        else:
            s = np.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
            qw = (r[1, 0] - r[0, 1]) / s
            qx = (r[0, 2] + r[2, 0]) / s
            qy = (r[1, 2] + r[2, 1]) / s
            qz = 0.25 * s
    return _normalize_quat(np.array([qw, qx, qy, qz]))


class DirectStateBridgeNode(Node):
    """
    Bypasses EKF by fusing Vicon Pose and IMU data directly.
    Provides a 13-element state array for the MPC controller.
    """
    def __init__(self):
        super().__init__('state_estimator_bridge')

        hostname = socket.gethostname()
        default_vicon_name = hostname if hostname in ('zephyrus', 'boreas') else 'zephyrus'
        
        self.declare_parameter('vicon_topic', f'/vicon/{default_vicon_name}/{default_vicon_name}')
        self.declare_parameter('imu_topic', '/imu0')
        self.declare_parameter('state_topic', 'drone_state_sim')
        self.declare_parameter('publish_rate', 100.0) # Hz
        self.declare_parameter('pose_world_frame', 'enu')
        self.declare_parameter('pose_body_frame', 'flu')
        self.declare_parameter('imu_body_frame', 'flu')
        
        vicon_topic = self.get_parameter('vicon_topic').value
        imu_topic = self.get_parameter('imu_topic').value
        state_topic = self.get_parameter('state_topic').value
        publish_rate = self.get_parameter('publish_rate').value
        self.pose_world_frame = self.get_parameter('pose_world_frame').value.lower()
        self.pose_body_frame = self.get_parameter('pose_body_frame').value.lower()
        self.imu_body_frame = self.get_parameter('imu_body_frame').value.lower()
        
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
        imu_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.imu_sub = self.create_subscription(
            Imu, imu_topic, self.imu_callback, imu_qos)
            
        # Publisher
        self.publisher = self.create_publisher(Float64MultiArray, state_topic, 10)
        
        # Timer for fixed-rate output
        self.timer = self.create_timer(1.0/publish_rate, self.timer_callback)
        
        self.get_logger().info(
            f'Direct State Bridge started: Vicon({vicon_topic}) + IMU({imu_topic}) '
            f'-> {state_topic} @ {publish_rate}Hz; pose={self.pose_world_frame}/{self.pose_body_frame} '
            f'-> ned/frd, imu_body={self.imu_body_frame} -> frd'
        )

    def _world_vector_to_ned(self, vector: np.ndarray) -> np.ndarray:
        if self.pose_world_frame == 'ned':
            return vector
        if self.pose_world_frame == 'enu':
            return _ENU_TO_NED @ vector
        self.get_logger().warn(
            f'Unknown pose_world_frame "{self.pose_world_frame}", assuming ENU')
        return _ENU_TO_NED @ vector

    def _body_vector_to_frd(self, vector: np.ndarray, body_frame: str) -> np.ndarray:
        if body_frame == 'frd':
            return vector
        if body_frame == 'flu':
            return _FLU_TO_FRD @ vector
        self.get_logger().warn(
            f'Unknown body frame "{body_frame}", assuming FLU')
        return _FLU_TO_FRD @ vector

    def _pose_quat_to_ned_frd(self, q_world_body: np.ndarray) -> np.ndarray:
        r_world_body = _quat_to_rot(q_world_body)
        world_transform = np.eye(3) if self.pose_world_frame == 'ned' else _ENU_TO_NED
        body_transform = np.eye(3) if self.pose_body_frame == 'frd' else _FLU_TO_FRD
        r_ned_frd = world_transform @ r_world_body @ body_transform.T
        return _rot_to_quat(r_ned_frd)

    def vicon_callback(self, msg: PoseStamped):
        raw_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        curr_pos = self._world_vector_to_ned(raw_pos)
        curr_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        # Simple velocity differentiation
        if self.prev_pos is not None and self.prev_time is not None:
            dt = curr_time - self.prev_time
            if dt > 0:
                raw_vel = (curr_pos - self.prev_pos) / dt
                # Low-pass filter (alpha=0.5) to handle noisy Vicon derivatives
                self.latest_vel = 0.5 * self.latest_vel + 0.5 * raw_vel
        
        self.latest_pos = curr_pos
        q_world_body = np.array([
            msg.pose.orientation.w,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z
        ])
        self.latest_quat = self._pose_quat_to_ned_frd(q_world_body)
        
        self.prev_pos = curr_pos
        self.prev_time = curr_time

    def imu_callback(self, msg: Imu):
        # Extract angular velocity directly from high-rate IMU
        omega = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
        self.latest_omega = self._body_vector_to_frd(omega, self.imu_body_frame)

    def timer_callback(self):
        # Construct 13-element state: [p(3), v(3), q(4), w(3)]
        # Note: MPC Controller expects [pos, vel, quat, omega]
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
