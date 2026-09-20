#!/usr/bin/env python3

from __future__ import annotations

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseWithCovarianceStamped
from scipy.spatial.transform import Rotation as R

from rl.aigp_state_estimator_runtime import (
    ErrorStateEstimator,
    ImuEstimatorState,
)


class AigpStateEstimatorNode(Node):
    def __init__(self):
        super().__init__("aigp_state_estimator_node")

        self.declare_parameter("imu_topic", "/imu0")
        self.declare_parameter("state_topic", "/drone_state_estimated")
        self.declare_parameter("set_pose_topic", "/aigp_state_estimator/set_pose")
        self.declare_parameter(
            "diagnostics_topic", "/aigp_state_estimator/diagnostics"
        )
        self.declare_parameter("publish_rate_hz", 100.0)
        self.declare_parameter("accel_is_specific_force", True)
        self.declare_parameter("velocity_damping_per_s", 0.0)
        self.declare_parameter("max_dt_s", 0.05)
        self.declare_parameter("accel_noise_std", 0.35)
        self.declare_parameter("gyro_noise_std", 0.03)
        self.declare_parameter("accel_bias_random_walk_std", 0.01)
        self.declare_parameter("gyro_bias_random_walk_std", 0.001)
        self.declare_parameter("vision_position_std_m", 0.25)
        self.declare_parameter("vision_orientation_std_rad", 0.1745329252)
        self.declare_parameter("vision_chi_square_threshold", 16.812)
        self.declare_parameter("history_duration_s", 0.5)
        self.declare_parameter("stationary_init_samples", 40)
        self.declare_parameter("stationary_max_gyro_rad_s", 0.15)
        self.declare_parameter("stationary_accel_tolerance_m_s2", 1.5)
        self.declare_parameter("initial_yaw_rad", 3.141592653589793)
        # Deprecated fixed-gain parameters remain declared so old launch files
        # do not fail parameter validation. They are intentionally not used.
        self.declare_parameter("fusion_alpha_pos", 0.3)
        self.declare_parameter("fusion_alpha_rot", 0.2)
        self.declare_parameter("fusion_alpha_vel", 0.2)
        self.declare_parameter("max_pos_err", 0.0)

        self.last_vision_time_s: float | None = None
        self.latest_state_msg: Float64MultiArray | None = None

        self.accel_is_specific_force = bool(self.get_parameter("accel_is_specific_force").value)
        self.velocity_damping_per_s = float(self.get_parameter("velocity_damping_per_s").value)
        self.max_dt_s = float(self.get_parameter("max_dt_s").value)
        self.max_pos_err = float(self.get_parameter("max_pos_err").value)
        self.estimator = ErrorStateEstimator(
            accel_is_specific_force=self.accel_is_specific_force,
            velocity_damping_per_s=self.velocity_damping_per_s,
            max_dt_s=self.max_dt_s,
            accel_noise_std=float(self.get_parameter("accel_noise_std").value),
            gyro_noise_std=float(self.get_parameter("gyro_noise_std").value),
            accel_bias_random_walk_std=float(
                self.get_parameter("accel_bias_random_walk_std").value
            ),
            gyro_bias_random_walk_std=float(
                self.get_parameter("gyro_bias_random_walk_std").value
            ),
            vision_position_std=float(self.get_parameter("vision_position_std_m").value),
            vision_orientation_std=float(
                self.get_parameter("vision_orientation_std_rad").value
            ),
            vision_chi_square_threshold=float(
                self.get_parameter("vision_chi_square_threshold").value
            ),
            max_position_error=self.max_pos_err,
            history_duration_s=float(self.get_parameter("history_duration_s").value),
        )
        self.stationary_init_samples = max(
            0, int(self.get_parameter("stationary_init_samples").value)
        )
        self.stationary_max_gyro_rad_s = float(
            self.get_parameter("stationary_max_gyro_rad_s").value
        )
        self.stationary_accel_tolerance_m_s2 = float(
            self.get_parameter("stationary_accel_tolerance_m_s2").value
        )
        self.initial_yaw_rad = float(self.get_parameter("initial_yaw_rad").value)
        self._stationary_initialized = self.stationary_init_samples == 0
        self._initial_imu_buffer: list[tuple[float, np.ndarray, np.ndarray]] = []

        self.imu_sub = self.create_subscription(
            Imu,
            str(self.get_parameter("imu_topic").value),
            self.imu_callback,
            50,
        )
        self.state_pub = self.create_publisher(
            Float64MultiArray,
            str(self.get_parameter("state_topic").value),
            10,
        )
        self.diagnostics_pub = self.create_publisher(
            Float64MultiArray,
            str(self.get_parameter("diagnostics_topic").value),
            10,
        )
        self.set_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter("set_pose_topic").value),
            self.set_pose_callback,
            10,
        )
        self.reset_srv = self.create_service(
            Trigger,
            "/aigp_state_estimator/reset",
            self.reset_callback,
        )

        rate_hz = float(self.get_parameter("publish_rate_hz").value)
        if rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be positive.")
        self.timer = self.create_timer(1.0 / rate_hz, self.publish_latest_state)

        self.get_logger().info(
            "AIGP covariance-aware IMU/vision ESKF publishing 13D NED/FRD state on "
            f"{self.get_parameter('state_topic').value}"
        )

    @property
    def state(self) -> ImuEstimatorState:
        """Expose the nominal state for backwards-compatible diagnostics."""
        return self.estimator.state

    def imu_callback(self, msg: Imu) -> None:
        stamp = msg.header.stamp
        now_s = stamp.sec + stamp.nanosec * 1e-9
        if now_s <= 0.0:
            now_s = self.get_clock().now().nanoseconds * 1e-9

        accel = np.array(
            [
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
            ],
            dtype=np.float64,
        )
        gyro = np.array(
            [
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z,
            ],
            dtype=np.float64,
        )

        if not self._stationary_initialized:
            self._initial_imu_buffer.append((now_s, accel, gyro))
            if self._stationary_window_is_invalid():
                self._stationary_initialized = True
                for buffered_time, buffered_accel, buffered_gyro in self._initial_imu_buffer:
                    self.estimator.process_imu(buffered_accel, buffered_gyro, buffered_time)
                self._initial_imu_buffer.clear()
                self.get_logger().info("Motion detected; skipped stationary IMU initialization.")
            elif len(self._initial_imu_buffer) >= self.stationary_init_samples:
                accelerometers = np.array([sample[1] for sample in self._initial_imu_buffer])
                gyroscopes = np.array([sample[2] for sample in self._initial_imu_buffer])
                initialized = self.estimator.initialize_stationary(
                    accelerometers,
                    gyroscopes,
                    initial_yaw_rad=self.initial_yaw_rad,
                )
                self._stationary_initialized = True
                if initialized:
                    last_time, last_accel, last_gyro = self._initial_imu_buffer[-1]
                    self.estimator.process_imu(last_accel, last_gyro, last_time)
                    self.get_logger().info("Initialized attitude and IMU biases while stationary.")
                self._initial_imu_buffer.clear()
            self.update_state_msg()
            return

        accepted = self.estimator.process_imu(accel, gyro, now_s)
        if not accepted:
            self.get_logger().debug("Ignored invalid or out-of-order IMU sample.")
        self.update_state_msg()

    def _stationary_window_is_invalid(self) -> bool:
        if not self._initial_imu_buffer:
            return False
        acceleration = self._initial_imu_buffer[-1][1]
        angular_velocity = self._initial_imu_buffer[-1][2]
        excessive_rotation = np.linalg.norm(angular_velocity) > self.stationary_max_gyro_rad_s
        unexpected_acceleration = False
        if self.accel_is_specific_force:
            unexpected_acceleration = (
                abs(np.linalg.norm(acceleration) - 9.80665)
                > self.stationary_accel_tolerance_m_s2
            )
        return bool(excessive_rotation or unexpected_acceleration)

    def _complete_initialization_before_vision(self) -> None:
        """Avoid overwriting a global visual attitude with later gravity alignment."""
        if self._stationary_initialized:
            return
        if len(self._initial_imu_buffer) >= 2 and not self._stationary_window_is_invalid():
            accelerometers = np.array([sample[1] for sample in self._initial_imu_buffer])
            gyroscopes = np.array([sample[2] for sample in self._initial_imu_buffer])
            self.estimator.initialize_stationary(
                accelerometers,
                gyroscopes,
                initial_yaw_rad=self.initial_yaw_rad,
            )
            last_time, last_accel, last_gyro = self._initial_imu_buffer[-1]
            self.estimator.process_imu(last_accel, last_gyro, last_time)
        else:
            for buffered_time, buffered_accel, buffered_gyro in self._initial_imu_buffer:
                self.estimator.process_imu(buffered_accel, buffered_gyro, buffered_time)
        self._initial_imu_buffer.clear()
        self._stationary_initialized = True

    def update_state_msg(self) -> None:
        msg = Float64MultiArray()
        msg.data = self.state.as_policy_state().tolist()
        self.latest_state_msg = msg

    def publish_latest_state(self) -> None:
        if self.latest_state_msg is not None:
            self.state_pub.publish(self.latest_state_msg)

    def publish_diagnostics(self, result=None) -> None:
        """Publish filter health without changing the policy's 13D state API.

        Layout: accepted, NIS, delayed, rewind_age_s, RMS position sigma,
        RMS velocity sigma, RMS attitude sigma, accel-bias norm, gyro-bias norm.
        """
        covariance_diag = np.maximum(np.diag(self.state.covariance), 0.0)
        accepted = -1.0 if result is None else float(result.accepted)
        nis = -1.0
        delayed = 0.0
        rewind_age_s = 0.0
        if result is not None:
            if np.isfinite(result.mahalanobis_squared):
                nis = float(result.mahalanobis_squared)
            delayed = float(result.delayed)
            if (
                result.applied_timestamp_s is not None
                and self.estimator.last_imu_time_s is not None
            ):
                rewind_age_s = max(
                    0.0,
                    self.estimator.last_imu_time_s - result.applied_timestamp_s,
                )
        msg = Float64MultiArray()
        msg.data = [
            accepted,
            nis,
            delayed,
            rewind_age_s,
            float(np.sqrt(np.mean(covariance_diag[0:3]))),
            float(np.sqrt(np.mean(covariance_diag[3:6]))),
            float(np.sqrt(np.mean(covariance_diag[6:9]))),
            float(np.linalg.norm(self.state.accel_bias_frd)),
            float(np.linalg.norm(self.state.gyro_bias_frd)),
        ]
        self.diagnostics_pub.publish(msg)

    def set_pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        stamp = msg.header.stamp
        now_s = stamp.sec + stamp.nanosec * 1e-9
        if now_s <= 0.0:
            now_s = self.get_clock().now().nanoseconds * 1e-9

        self.last_vision_time_s = now_s

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        pos_vis_ned = np.array([p.x, p.y, p.z], dtype=np.float64)
        quaternion = np.array([q.x, q.y, q.z, q.w], dtype=np.float64)
        if not np.all(np.isfinite(quaternion)) or np.linalg.norm(quaternion) < 1e-9:
            self.get_logger().debug("Rejected vision pose with invalid quaternion.")
            return
        rot_vis = R.from_quat(quaternion)
        pose_covariance = np.asarray(msg.pose.covariance, dtype=np.float64).reshape(6, 6)

        self._complete_initialization_before_vision()
        result = self.estimator.update_vision(
            pos_vis_ned,
            rot_vis,
            covariance=pose_covariance,
            timestamp_s=now_s,
        )
        self.update_state_msg()
        self.publish_diagnostics(result)
        if result.accepted:
            self.get_logger().debug(
                f"Fused vision pose (NIS={result.mahalanobis_squared:.2f}, "
                f"delayed={result.delayed}): pos=[{self.state.position_ned[0]:.3f}, "
                f"{self.state.position_ned[1]:.3f}, {self.state.position_ned[2]:.3f}]"
            )
        else:
            self.get_logger().debug(
                f"Rejected vision pose ({result.reason}, NIS={result.mahalanobis_squared:.2f})."
            )

    def reset_callback(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        self.estimator.reset()
        self.last_vision_time_s = None
        self._stationary_initialized = self.stationary_init_samples == 0
        self._initial_imu_buffer.clear()
        self.update_state_msg()
        self.publish_diagnostics()
        response.success = True
        response.message = "AIGP IMU state estimator reset."
        return response


def main(args=None):
    rclpy.init(args=args)
    node = AigpStateEstimatorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
