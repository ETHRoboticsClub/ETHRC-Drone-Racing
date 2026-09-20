import unittest

import numpy as np
from scipy.spatial.transform import Rotation as R

from rl.aigp_state_estimator_runtime import (
    GRAVITY_NED,
    ErrorStateEstimator,
    ImuEstimatorState,
    integrate_imu_state,
    kalman_update_vision_pose,
    make_pose_covariance,
)


class TestAigpStateEstimatorRuntime(unittest.TestCase):
    def test_zero_state_matches_policy_state_shape_and_convention(self):
        state = ImuEstimatorState.zero().as_policy_state()

        self.assertEqual(state.shape, (13,))
        np.testing.assert_allclose(state[0:6], np.zeros(6))
        np.testing.assert_allclose(state[6:10], [1.0, 0.0, 0.0, 0.0])
        np.testing.assert_allclose(state[10:13], np.zeros(3))

    def test_stationary_specific_force_does_not_create_velocity(self):
        state = ImuEstimatorState.zero()

        next_state = integrate_imu_state(
            state,
            linear_acceleration_frd=-GRAVITY_NED,
            angular_velocity_frd=np.zeros(3),
            dt_s=0.01,
        )

        np.testing.assert_allclose(next_state.velocity_ned, np.zeros(3), atol=1e-12)
        np.testing.assert_allclose(next_state.position_ned, np.zeros(3), atol=1e-12)
        np.testing.assert_allclose(next_state.covariance, next_state.covariance.T)
        self.assertGreaterEqual(np.linalg.eigvalsh(next_state.covariance).min(), -1e-12)

    def test_body_gyro_integrates_orientation_and_removes_bias(self):
        state = ImuEstimatorState.zero()
        state.gyro_bias_frd = np.array([0.1, 0.0, 0.0])

        next_state = integrate_imu_state(
            state,
            linear_acceleration_frd=-GRAVITY_NED,
            angular_velocity_frd=np.array([0.1, 0.0, 0.0]),
            dt_s=0.1,
            max_dt_s=0.2,
        )

        np.testing.assert_allclose(next_state.angular_velocity_frd, np.zeros(3), atol=1e-12)
        np.testing.assert_allclose(next_state.rotation_ned_from_frd.as_rotvec(), np.zeros(3))

    def test_dt_is_capped_for_large_timestamp_gaps(self):
        state = ImuEstimatorState.zero()

        next_state = integrate_imu_state(
            state,
            linear_acceleration_frd=np.array([1.0, 0.0, 0.0]),
            angular_velocity_frd=np.zeros(3),
            dt_s=1.0,
            accel_is_specific_force=False,
            max_dt_s=0.05,
        )

        np.testing.assert_allclose(next_state.velocity_ned, [0.05, 0.0, 0.0])

    def test_pose_covariance_uses_fallback_for_unknown_ros_covariance(self):
        covariance = make_pose_covariance(np.zeros((6, 6)), 0.4, 0.2)

        np.testing.assert_allclose(np.diag(covariance)[0:3], [0.16] * 3)
        np.testing.assert_allclose(np.diag(covariance)[3:6], [0.04] * 3)

    def test_covariance_aware_pose_update_reduces_uncertainty(self):
        state = ImuEstimatorState.zero()
        measurement_covariance = np.diag([0.1**2] * 3 + [0.05**2] * 3)

        corrected, result = kalman_update_vision_pose(
            state,
            np.array([1.0, -0.2, 0.1]),
            R.from_euler("z", 0.1),
            measurement_covariance,
        )

        self.assertTrue(result.accepted)
        self.assertGreater(corrected.position_ned[0], 0.9)
        self.assertLess(corrected.covariance[0, 0], state.covariance[0, 0])
        self.assertLess(corrected.covariance[8, 8], state.covariance[8, 8])

    def test_chi_square_gate_rejects_inconsistent_pose_without_mutation(self):
        state = ImuEstimatorState.zero()
        state.covariance = np.eye(15) * 1e-3
        measurement_covariance = np.eye(6) * 1e-3

        corrected, result = kalman_update_vision_pose(
            state,
            np.array([10.0, 0.0, 0.0]),
            R.identity(),
            measurement_covariance,
        )

        self.assertFalse(result.accepted)
        self.assertEqual(result.reason, "chi-square gate")
        self.assertIs(corrected, state)

    def test_delayed_vision_replay_matches_on_time_update(self):
        kwargs = dict(
            accel_is_specific_force=False,
            velocity_damping_per_s=0.0,
            max_dt_s=0.2,
            history_duration_s=1.0,
        )
        on_time = ErrorStateEstimator(**kwargs)
        delayed = ErrorStateEstimator(**kwargs)
        acceleration = np.array([0.5, 0.0, 0.0])
        angular_velocity = np.array([0.0, 0.0, 0.1])
        pose_position = np.array([0.02, -0.01, 0.0])
        pose_rotation = R.from_euler("z", 0.015)

        for estimator in (on_time, delayed):
            estimator.process_imu(acceleration, angular_velocity, 0.0)
            estimator.process_imu(acceleration, angular_velocity, 0.1)
        on_time_result = on_time.update_vision(
            pose_position, pose_rotation, timestamp_s=0.1
        )
        on_time.process_imu(acceleration, angular_velocity, 0.2)

        delayed.process_imu(acceleration, angular_velocity, 0.2)
        delayed_result = delayed.update_vision(
            pose_position, pose_rotation, timestamp_s=0.1
        )

        self.assertTrue(on_time_result.accepted)
        self.assertTrue(delayed_result.accepted)
        self.assertTrue(delayed_result.delayed)
        np.testing.assert_allclose(
            delayed.state.as_policy_state(), on_time.state.as_policy_state(), atol=1e-12
        )
        np.testing.assert_allclose(delayed.state.covariance, on_time.state.covariance, atol=1e-12)

    def test_stationary_initialization_estimates_imu_biases(self):
        estimator = ErrorStateEstimator()
        accel_bias = np.array([0.02, -0.03, 0.08])
        gyro_bias = np.array([0.01, -0.02, 0.005])
        accelerometers = np.tile(-GRAVITY_NED + accel_bias, (50, 1))
        gyroscopes = np.tile(gyro_bias, (50, 1))

        initialized = estimator.initialize_stationary(accelerometers, gyroscopes)

        self.assertTrue(initialized)
        np.testing.assert_allclose(estimator.state.gyro_bias_frd, gyro_bias)
        # Roll/pitch absorbs cross-axis acceleration; the remaining magnitude
        # residual is correctly represented as an accelerometer bias.
        corrected_specific_force = accelerometers[0] - estimator.state.accel_bias_frd
        np.testing.assert_allclose(
            estimator.state.rotation_ned_from_frd.apply(corrected_specific_force),
            -GRAVITY_NED,
            atol=1e-10,
        )

    def test_stationary_initialization_applies_known_start_yaw(self):
        estimator = ErrorStateEstimator()
        accelerometers = np.tile(-GRAVITY_NED, (50, 1))
        gyroscopes = np.zeros((50, 3))

        initialized = estimator.initialize_stationary(
            accelerometers,
            gyroscopes,
            initial_yaw_rad=np.pi,
        )

        self.assertTrue(initialized)
        body_forward_ned = estimator.state.rotation_ned_from_frd.apply([1.0, 0.0, 0.0])
        np.testing.assert_allclose(body_forward_ned, [-1.0, 0.0, 0.0], atol=1e-12)


if __name__ == "__main__":
    unittest.main()
