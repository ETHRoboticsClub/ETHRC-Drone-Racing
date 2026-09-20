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

try:
    import gz.transport13 as gz_transport
    import gz.msgs10.imu_pb2 as gz_imu_msg
    import gz.msgs10.odometry_pb2 as gz_odom_msg
except Exception:
    gz_transport = None
    gz_imu_msg = None
    gz_odom_msg = None


def gazebo_message_time_seconds(message) -> float | None:
    """Return a positive Gazebo simulation timestamp when one is present.

    Gazebo Transport callbacks are delivered on a wall-clock thread.  Using
    ``time.monotonic()`` to differentiate successive poses therefore makes
    the velocity seen by the policy depend on real-time factor and callback
    jitter, rather than on the physics trajectory.  Isaac supplies simulated
    body velocity, so this is a direct deployment-observation mismatch.

    The helper intentionally accepts a duck-typed protobuf object to keep the
    state bridge compatible with the message versions shipped by Gazebo
    Fortress/Harmonic.  A missing or zero timestamp remains a safe fallback to
    wall time in the caller.
    """
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return None

    try:
        seconds = float(getattr(stamp, "sec", 0))
        nanoseconds = float(getattr(stamp, "nsec", 0))
    except (TypeError, ValueError):
        return None

    timestamp = seconds + nanoseconds * 1e-9
    return timestamp if np.isfinite(timestamp) and timestamp > 0.0 else None

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
        self.declare_parameter('gazebo_imu_topic', '/drone/imu')
        self.declare_parameter('gazebo_odometry_topic', '/drone/odometry')
        self.declare_parameter('use_gazebo_transport_fallback', True)
        self.declare_parameter('state_topic', 'drone_state_sim')
        self.declare_parameter('publish_rate', 100.0) # Hz
        
        self.pose_source = self.get_parameter('pose_source').value
        vicon_topic = self.get_parameter('vicon_topic').value
        odometry_topic = self.get_parameter('odometry_topic').value
        imu_topic = self.get_parameter('imu_topic').value
        gazebo_imu_topic = self.get_parameter('gazebo_imu_topic').value
        gazebo_odometry_topic = self.get_parameter('gazebo_odometry_topic').value
        use_gazebo_transport_fallback = self.get_parameter('use_gazebo_transport_fallback').value
        state_topic = self.get_parameter('state_topic').value
        publish_rate = self.get_parameter('publish_rate').value
        self._logged_first_state = False
        
        # Latest data buffers
        self.latest_pos = np.zeros(3)
        self.latest_quat = np.array([0.0, 0.0, 0.0, 1.0])
        self.latest_vel = np.zeros(3)
        self.latest_omega = np.zeros(3)
        self.has_vicon_data = False # Reused as has_pose_data
        self.has_imu_data = False
        
        # Velocity estimation state. Gazebo transport odometry can report zero
        # twist while pose is changing, so keep a pose-difference fallback.
        self.prev_pos = None
        self.prev_time = None
        
        # Subscriptions
        if self.pose_source == 'vicon':
            self.vicon_sub = self.create_subscription(
                PoseStamped, vicon_topic, self.vicon_callback, 10)
            self.get_logger().info(f'Subscribed to Vicon: {vicon_topic}')
        elif self.pose_source == 'odometry':
            from rclpy.qos import QoSProfile, ReliabilityPolicy
            odom_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
            self.odom_sub = self.create_subscription(
                Odometry, odometry_topic, self.odom_callback, odom_qos)
            self.get_logger().info(f'Subscribed to Odometry: {odometry_topic}')
        else:
            self.get_logger().error(f'Invalid pose_source: {self.pose_source}')
        
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        self.imu_sub = self.create_subscription(
            Imu, imu_topic, self.imu_callback,
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10))

        self.gz_node = None
        if self.pose_source == 'odometry' and use_gazebo_transport_fallback:
            self._setup_gazebo_transport_fallback(gazebo_odometry_topic, gazebo_imu_topic)
            
        # Publisher
        self.publisher = self.create_publisher(Float64MultiArray, state_topic, 10)
        
        # Timer for fixed-rate output
        self.timer = self.create_timer(1.0/publish_rate, self.timer_callback)
        
        self.get_logger().info(f'Direct State Bridge started: Source({self.pose_source}) + IMU({imu_topic}) -> {state_topic} @ {publish_rate}Hz')

    def _setup_gazebo_transport_fallback(self, odometry_topic: str, imu_topic: str):
        if gz_transport is None or gz_imu_msg is None or gz_odom_msg is None:
            self.get_logger().warning(
                "Gazebo transport Python bindings unavailable; relying on ROS /odometry and /imu0."
            )
            return

        self.gz_node = gz_transport.Node()
        self.gz_node.subscribe(gz_odom_msg.Odometry, odometry_topic, self.gz_odom_callback)
        self.gz_node.subscribe(gz_imu_msg.IMU, imu_topic, self.gz_imu_callback)
        self.get_logger().info(
            f'Gazebo transport fallback subscribed: {odometry_topic} + {imu_topic}'
        )

    def gz_odom_callback(self, msg):
        curr_pos = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])

        self.latest_quat = np.array([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ])

        vel_msg = np.array([
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.linear.z
        ])
        # Differentiate positions in simulation time, not callback wall time.
        # The latter changes whenever Gazebo real-time factor changes and was
        # feeding a velocity signal unlike the one used for Isaac training.
        sample_time = gazebo_message_time_seconds(msg)
        self.latest_vel = self._select_velocity(
            curr_pos,
            sample_time if sample_time is not None else time.monotonic(),
            vel_msg,
        )
        self.latest_pos = curr_pos
        self.has_vicon_data = True

    def _select_velocity(self, curr_pos: np.ndarray, curr_time: float, measured_vel: np.ndarray | None = None) -> np.ndarray:
        raw_vel = None
        if self.prev_pos is not None and self.prev_time is not None:
            dt = curr_time - self.prev_time
            if 1e-4 < dt < 1.0:
                raw_vel = (curr_pos - self.prev_pos) / dt

        self.prev_pos = curr_pos.copy()
        self.prev_time = curr_time

        if measured_vel is not None and np.isfinite(measured_vel).all():
            if raw_vel is None:
                return measured_vel
            if np.linalg.norm(measured_vel) > 1e-4 or np.linalg.norm(raw_vel) < 1e-4:
                return measured_vel

        if raw_vel is None or not np.isfinite(raw_vel).all():
            return self.latest_vel
        return raw_vel

    def gz_imu_callback(self, msg):
        omega = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
        # The SITL IMU is mounted in aircraft FRD for Betaflight, while the
        # controller state and odometry attitude use ROS FLU.
        self.latest_omega = omega * np.array([1.0, -1.0, -1.0])
        self.has_imu_data = True

    def vicon_callback(self, msg: PoseStamped):
        curr_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        curr_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.latest_vel = self._select_velocity(curr_pos, curr_time)
        
        self.latest_pos = curr_pos
        self.latest_quat = np.array([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ])
        self.has_vicon_data = True

    def odom_callback(self, msg: Odometry):
        curr_pos = np.array([
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
            vel_msg = r.apply(vel_msg)
        else:
            vel_msg = vel_msg

        curr_time = time.monotonic()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp > 0.0:
            curr_time = stamp
        self.latest_vel = self._select_velocity(curr_pos, curr_time, vel_msg)
        self.latest_pos = curr_pos
            
        self.has_vicon_data = True

    def imu_callback(self, msg: Imu):
        # Extract angular velocity directly from high-rate IMU
        omega = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
        if self.pose_source == 'odometry':
            omega = omega * np.array([1.0, -1.0, -1.0])
        self.latest_omega = omega
        self.has_imu_data = True

    def timer_callback(self):
        if not self.has_vicon_data or not self.has_imu_data:
            self.get_logger().warning(
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
        
        # Sanity check for NaNs and Infs
        if not np.isfinite(data).all():
            return

        state_msg = Float64MultiArray()
        state_msg.data = data
        self.publisher.publish(state_msg)
        if not self._logged_first_state:
            self.get_logger().info("Publishing state to controller.")
            self._logged_first_state = True

def main(args=None):
    rclpy.init(args=args)
    node = DirectStateBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
