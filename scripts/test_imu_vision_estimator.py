#!/usr/bin/env python3
"""
Test script verifying state estimation without ground truth.
Tests IMU dead-reckoning (/imu0) fused with neon orange gate vision pose (/aigp_state_estimator/set_pose)
to produce the estimated drone state (/drone_state_estimated).
"""

import sys
import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float64MultiArray
from scipy.spatial.transform import Rotation as R

from rl.aigp_state_estimator_node import AigpStateEstimatorNode


class EstimatorTestVerifier(Node):
    def __init__(self):
        super().__init__('estimator_test_verifier')

        self.estimated_states = []

        self.sub = self.create_subscription(
            Float64MultiArray,
            '/drone_state_estimated',
            self.state_callback,
            10
        )

        self.imu_pub = self.create_publisher(Imu, '/imu0', 10)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/aigp_state_estimator/set_pose', 10)

    def state_callback(self, msg: Float64MultiArray):
        if len(msg.data) >= 13:
            self.estimated_states.append(np.array(msg.data[:13], dtype=np.float64))

    def publish_test_imu(self, dt=0.01):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"
        # Hovering specific force: +9.80665 m/s^2 upward in FRD (or -9.80665 m/s^2 zaccel)
        msg.linear_acceleration.x = 0.0
        msg.linear_acceleration.y = 0.0
        msg.linear_acceleration.z = -9.80665
        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = 0.0
        self.imu_pub.publish(msg)

    def publish_test_gate_pose(self, x=0.0, y=0.0, z=-1.0):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = float(z)
        msg.pose.pose.orientation.w = 1.0
        self.pose_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    estimator_node = AigpStateEstimatorNode()
    verifier_node = EstimatorTestVerifier()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(estimator_node)
    executor.add_node(verifier_node)

    print("Starting No-Ground-Truth State Estimation Test...")
    start_time = time.time()

    # Step 1: Simulate 1 second of stationary IMU integration at 100Hz
    for _ in range(100):
        verifier_node.publish_test_imu(dt=0.01)
        executor.spin_once(timeout_sec=0.01)
        time.sleep(0.005)

    print(f"Received {len(verifier_node.estimated_states)} estimated state messages during IMU integration.")
    assert len(verifier_node.estimated_states) > 0, "No state messages received!"

    initial_estimate = verifier_node.estimated_states[-1]
    print(f"Initial IMU-only estimated pose: NED=[{initial_estimate[0]:.3f}, {initial_estimate[1]:.3f}, {initial_estimate[2]:.3f}]")

    # Step 2: Inject visual gate detection pose correction (hovering at z = -1.0m)
    print("Injecting neon orange gate detection pose correction at NED=(0, 0, -1.0)...")
    for _ in range(10):
        verifier_node.publish_test_gate_pose(x=0.0, y=0.0, z=-1.0)
        verifier_node.publish_test_imu(dt=0.01)
        executor.spin_once(timeout_sec=0.02)
        time.sleep(0.01)

    # Allow timer to publish updated state
    for _ in range(20):
        executor.spin_once(timeout_sec=0.02)
        time.sleep(0.01)

    fused_estimate = verifier_node.estimated_states[-1]
    print(f"Fused state estimated pose: NED=[{fused_estimate[0]:.3f}, {fused_estimate[1]:.3f}, {fused_estimate[2]:.3f}]")

    # Verify fusion moved state towards gate observation (z -> -1.0m)
    assert fused_estimate[2] < -0.1, f"Fusion failed to update altitude towards -1.0m, got {fused_estimate[2]}"
    print("=== NO-GROUND-TRUTH STATE ESTIMATOR VERIFICATION SUCCESSFUL ===")

    estimator_node.destroy_node()
    verifier_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
