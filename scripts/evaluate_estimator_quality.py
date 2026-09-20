#!/usr/bin/env python3
"""
Evaluates the quality of the state estimator (/drone_state_estimated)
by comparing it against ground-truth telemetry (/drone_state_sim).
Calculates RMSE, Max Error, and Attitude Error over a test duration.
"""

import time
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from scipy.spatial.transform import Rotation as R


class EstimatorQualityEvaluator(Node):
    def __init__(self):
        super().__init__('estimator_quality_evaluator')

        self.declare_parameter('gt_topic', '/drone_state_sim')
        self.declare_parameter('est_topic', '/drone_state_estimated')
        self.declare_parameter('duration_s', 10.0)

        gt_topic = self.get_parameter('gt_topic').value
        est_topic = self.get_parameter('est_topic').value
        self.duration_s = float(self.get_parameter('duration_s').value)

        self.gt_history = []
        self.est_history = []

        self.gt_sub = self.create_subscription(
            Float64MultiArray, gt_topic, self.gt_callback, 10
        )
        self.est_sub = self.create_subscription(
            Float64MultiArray, est_topic, self.est_callback, 10
        )

        self.start_time = time.time()
        self.timer = self.create_timer(0.1, self.check_completion)
        self.get_logger().info(f"Evaluating estimator quality against GT for {self.duration_s}s...")

    def gt_callback(self, msg: Float64MultiArray):
        if len(msg.data) >= 13:
            self.gt_history.append((time.time(), np.array(msg.data[:13], dtype=np.float64)))

    def est_callback(self, msg: Float64MultiArray):
        if len(msg.data) >= 13:
            self.est_history.append((time.time(), np.array(msg.data[:13], dtype=np.float64)))

    def check_completion(self):
        if time.time() - self.start_time >= self.duration_s:
            self.compute_metrics()
            rclpy.shutdown()

    def compute_metrics(self):
        if not self.gt_history or not self.est_history:
            self.get_logger().error("Insufficient telemetry samples collected.")
            return

        # Time-align GT and Estimated samples using nearest timestamp
        gt_times = np.array([t for t, _ in self.gt_history])
        pos_errors = []
        vel_errors = []
        rot_errors_deg = []

        for est_t, est_state in self.est_history:
            idx = np.argmin(np.abs(gt_times - est_t))
            if np.abs(gt_times[idx] - est_t) > 0.05:
                continue  # skip misaligned samples

            gt_state = self.gt_history[idx][1]

            # Position error
            p_err = np.linalg.norm(est_state[:3] - gt_state[:3])
            pos_errors.append(p_err)

            # Velocity error
            v_err = np.linalg.norm(est_state[3:6] - gt_state[3:6])
            vel_errors.append(v_err)

            # Attitude error (quaternion angular difference)
            q_est = R.from_quat([est_state[7], est_state[8], est_state[9], est_state[6]]) # xyzw
            q_gt  = R.from_quat([gt_state[7], gt_state[8], gt_state[9], gt_state[6]])
            q_rel = q_est * q_gt.inv()
            angle_deg = np.degrees(q_rel.magnitude())
            rot_errors_deg.append(angle_deg)

        if not pos_errors:
            self.get_logger().error("No time-aligned samples found!")
            return

        pos_errors = np.array(pos_errors)
        vel_errors = np.array(vel_errors)
        rot_errors_deg = np.array(rot_errors_deg)

        pos_rmse = np.sqrt(np.mean(pos_errors ** 2))
        vel_rmse = np.sqrt(np.mean(vel_errors ** 2))
        rot_rmse_deg = np.sqrt(np.mean(rot_errors_deg ** 2))

        print("\n==================================================")
        print("   STATE ESTIMATOR QUALITY VS GROUND TRUTH REPORT")
        print("==================================================")
        print(f"Total Evaluated Samples: {len(pos_errors)}")
        print("--------------------------------------------------")
        print(f"Position RMSE:    {pos_rmse * 100.0:.2f} cm  (Max: {np.max(pos_errors) * 100.0:.2f} cm)")
        print(f"Velocity RMSE:    {vel_rmse:.3f} m/s  (Max: {np.max(vel_errors):.3f} m/s)")
        print(f"Attitude RMSE:    {rot_rmse_deg:.2f}°     (Max: {np.max(rot_errors_deg):.2f}°)")
        print("==================================================\n")


def main(args=None):
    rclpy.init(args=args)
    node = EstimatorQualityEvaluator()
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
