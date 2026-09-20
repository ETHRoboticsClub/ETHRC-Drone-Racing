"""Bias-aware inertial/visual state estimation used by the AIGP runtime.

The nominal state is position, velocity, attitude, accelerometer bias and gyro
bias.  Its 15 dimensional local error is ordered ``[dp, dv, dtheta, dba,
dbg]``.  Attitude errors are right-multiplicative (body frame).
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp


GRAVITY_NED = np.array([0.0, 0.0, 9.80665], dtype=np.float64)
ERROR_STATE_SIZE = 15


def _initial_covariance() -> np.ndarray:
    # Position is deliberately broad so the first global gate fix is accepted.
    std = np.array(
        [5.0] * 3
        + [1.0] * 3
        + [np.deg2rad(30.0)] * 3
        + [0.5] * 3
        + [np.deg2rad(5.0)] * 3,
        dtype=np.float64,
    )
    return np.diag(std * std)


def _skew(vector: np.ndarray) -> np.ndarray:
    x, y, z = np.asarray(vector, dtype=np.float64)
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def _shortest_rotation_between(source: np.ndarray, target: np.ndarray) -> R:
    """Return the zero-twist rotation mapping ``source`` onto ``target``.

    A single gravity vector cannot determine yaw. Choosing the shortest arc is
    deterministic and avoids ``align_vectors`` inventing an arbitrary yaw.
    """
    source_unit = np.array(source, dtype=np.float64, copy=True)
    target_unit = np.array(target, dtype=np.float64, copy=True)
    source_unit /= np.linalg.norm(source_unit)
    target_unit /= np.linalg.norm(target_unit)
    dot = float(np.clip(np.dot(source_unit, target_unit), -1.0, 1.0))
    if dot > 1.0 - 1e-12:
        return R.identity()
    if dot < -1.0 + 1e-12:
        basis = np.eye(3)[int(np.argmin(np.abs(source_unit)))]
        axis = np.cross(source_unit, basis)
        axis /= np.linalg.norm(axis)
        return R.from_rotvec(np.pi * axis)
    quaternion_xyzw = np.concatenate(
        [np.cross(source_unit, target_unit), np.array([1.0 + dot])]
    )
    quaternion_xyzw /= np.linalg.norm(quaternion_xyzw)
    return R.from_quat(quaternion_xyzw)


@dataclass
class ImuEstimatorState:
    position_ned: np.ndarray
    velocity_ned: np.ndarray
    rotation_ned_from_frd: R
    angular_velocity_frd: np.ndarray
    accel_bias_frd: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    gyro_bias_frd: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    covariance: np.ndarray = field(default_factory=_initial_covariance)
    previous_acceleration_frd: np.ndarray | None = None
    previous_angular_velocity_frd: np.ndarray | None = None

    @classmethod
    def zero(cls) -> "ImuEstimatorState":
        return cls(
            position_ned=np.zeros(3, dtype=np.float64),
            velocity_ned=np.zeros(3, dtype=np.float64),
            rotation_ned_from_frd=R.identity(),
            angular_velocity_frd=np.zeros(3, dtype=np.float64),
        )

    def copy(self) -> "ImuEstimatorState":
        return ImuEstimatorState(
            position_ned=self.position_ned.copy(),
            velocity_ned=self.velocity_ned.copy(),
            rotation_ned_from_frd=R.from_quat(self.rotation_ned_from_frd.as_quat()),
            angular_velocity_frd=self.angular_velocity_frd.copy(),
            accel_bias_frd=self.accel_bias_frd.copy(),
            gyro_bias_frd=self.gyro_bias_frd.copy(),
            covariance=self.covariance.copy(),
            previous_acceleration_frd=(
                None
                if self.previous_acceleration_frd is None
                else self.previous_acceleration_frd.copy()
            ),
            previous_angular_velocity_frd=(
                None
                if self.previous_angular_velocity_frd is None
                else self.previous_angular_velocity_frd.copy()
            ),
        )

    def as_policy_state(self) -> np.ndarray:
        """Return the backwards-compatible [p, v, q_wxyz, omega] API."""
        quat_xyzw = self.rotation_ned_from_frd.as_quat()
        quat_wxyz = np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]])
        return np.concatenate(
            [
                self.position_ned,
                self.velocity_ned,
                quat_wxyz,
                self.angular_velocity_frd,
            ]
        ).astype(np.float64)


def integrate_imu_state(
    state: ImuEstimatorState,
    linear_acceleration_frd: np.ndarray,
    angular_velocity_frd: np.ndarray,
    dt_s: float,
    *,
    accel_is_specific_force: bool = True,
    velocity_damping_per_s: float = 0.0,
    max_dt_s: float = 0.05,
    accel_noise_std: float = 0.35,
    gyro_noise_std: float = 0.03,
    accel_bias_random_walk_std: float = 0.01,
    gyro_bias_random_walk_std: float = 0.001,
) -> ImuEstimatorState:
    """Propagate the nominal state and 15x15 covariance with one IMU sample.

    The current and previous samples are averaged when possible.  Large or
    invalid timestamp gaps are capped to prevent a single delayed packet from
    destabilizing the estimate.
    """
    if dt_s <= 0.0:
        return state
    if max_dt_s <= 0.0:
        raise ValueError("max_dt_s must be positive")

    dt = min(float(dt_s), float(max_dt_s))
    accel_raw = np.asarray(linear_acceleration_frd, dtype=np.float64)
    gyro_raw = np.asarray(angular_velocity_frd, dtype=np.float64)
    if accel_raw.shape != (3,):
        raise ValueError(f"linear_acceleration_frd must have shape (3,), got {accel_raw.shape}")
    if gyro_raw.shape != (3,):
        raise ValueError(f"angular_velocity_frd must have shape (3,), got {gyro_raw.shape}")
    if not np.all(np.isfinite(accel_raw)) or not np.all(np.isfinite(gyro_raw)):
        raise ValueError("IMU samples must be finite")

    previous_accel = (
        accel_raw if state.previous_acceleration_frd is None else state.previous_acceleration_frd
    )
    previous_gyro = (
        gyro_raw
        if state.previous_angular_velocity_frd is None
        else state.previous_angular_velocity_frd
    )
    accel_body = 0.5 * (previous_accel + accel_raw) - state.accel_bias_frd
    gyro_body = 0.5 * (previous_gyro + gyro_raw) - state.gyro_bias_frd

    half_rotation = state.rotation_ned_from_frd * R.from_rotvec(0.5 * gyro_body * dt)
    next_rotation = state.rotation_ned_from_frd * R.from_rotvec(gyro_body * dt)
    accel_ned = half_rotation.apply(accel_body)
    if accel_is_specific_force:
        accel_ned = accel_ned + GRAVITY_NED

    next_position = state.position_ned + state.velocity_ned * dt + 0.5 * accel_ned * dt * dt
    next_velocity = state.velocity_ned + accel_ned * dt
    damping = 1.0
    if velocity_damping_per_s > 0.0:
        damping = np.exp(-float(velocity_damping_per_s) * dt)
        next_velocity *= damping

    # First-order continuous error dynamics, discretized over this short IMU step.
    transition_rate = np.zeros((ERROR_STATE_SIZE, ERROR_STATE_SIZE), dtype=np.float64)
    transition_rate[0:3, 3:6] = np.eye(3)
    rotation_matrix = half_rotation.as_matrix()
    transition_rate[3:6, 6:9] = -rotation_matrix @ _skew(accel_body)
    transition_rate[3:6, 9:12] = -rotation_matrix
    transition_rate[6:9, 6:9] = -_skew(gyro_body)
    transition_rate[6:9, 12:15] = -np.eye(3)
    transition = np.eye(ERROR_STATE_SIZE) + transition_rate * dt
    transition[3:6, :] *= damping

    noise_jacobian = np.zeros((ERROR_STATE_SIZE, 12), dtype=np.float64)
    noise_jacobian[3:6, 0:3] = damping * rotation_matrix
    noise_jacobian[6:9, 3:6] = np.eye(3)
    noise_jacobian[9:12, 6:9] = np.eye(3)
    noise_jacobian[12:15, 9:12] = np.eye(3)
    noise_variances = np.array(
        [accel_noise_std**2] * 3
        + [gyro_noise_std**2] * 3
        + [accel_bias_random_walk_std**2] * 3
        + [gyro_bias_random_walk_std**2] * 3,
        dtype=np.float64,
    )
    process_covariance = noise_jacobian @ np.diag(noise_variances) @ noise_jacobian.T * dt
    covariance = transition @ state.covariance @ transition.T + process_covariance
    covariance = 0.5 * (covariance + covariance.T)

    return ImuEstimatorState(
        position_ned=next_position,
        velocity_ned=next_velocity,
        rotation_ned_from_frd=next_rotation,
        angular_velocity_frd=gyro_raw - state.gyro_bias_frd,
        accel_bias_frd=state.accel_bias_frd.copy(),
        gyro_bias_frd=state.gyro_bias_frd.copy(),
        covariance=covariance,
        previous_acceleration_frd=accel_raw.copy(),
        previous_angular_velocity_frd=gyro_raw.copy(),
    )


def make_pose_covariance(
    covariance: np.ndarray | None,
    default_position_std: float,
    default_orientation_std: float,
) -> np.ndarray:
    """Validate a ROS-order 6x6 pose covariance, filling unknown entries."""
    default_variances = np.array(
        [default_position_std**2] * 3 + [default_orientation_std**2] * 3,
        dtype=np.float64,
    )
    if covariance is None:
        return np.diag(default_variances)
    matrix = np.asarray(covariance, dtype=np.float64)
    if matrix.size != 36:
        raise ValueError(f"pose covariance must contain 36 elements, got {matrix.size}")
    matrix = matrix.reshape(6, 6)
    if not np.all(np.isfinite(matrix)) or np.allclose(matrix, 0.0) or matrix[0, 0] < 0.0:
        return np.diag(default_variances)
    matrix = 0.5 * (matrix + matrix.T)
    for index, default_variance in enumerate(default_variances):
        if matrix[index, index] <= 0.0:
            matrix[index, index] = default_variance
    # Numerical/non-PSD user covariances should not break the Kalman solve.
    eigenvalues, eigenvectors = np.linalg.eigh(matrix)
    eigenvalues = np.maximum(eigenvalues, 1e-10)
    return eigenvectors @ np.diag(eigenvalues) @ eigenvectors.T


@dataclass(frozen=True)
class VisionUpdateResult:
    accepted: bool
    mahalanobis_squared: float
    reason: str
    delayed: bool = False
    applied_timestamp_s: float | None = None


def kalman_update_vision_pose(
    state: ImuEstimatorState,
    pos_vis_ned: np.ndarray,
    rot_vis_ned_from_frd: R,
    measurement_covariance: np.ndarray,
    *,
    chi_square_threshold: float = 16.812,
    max_position_error: float = 0.0,
) -> tuple[ImuEstimatorState, VisionUpdateResult]:
    """Fuse a 6-DoF pose using a covariance-aware error-state EKF update."""
    position = np.asarray(pos_vis_ned, dtype=np.float64)
    if position.shape != (3,) or not np.all(np.isfinite(position)):
        return state, VisionUpdateResult(False, np.inf, "invalid position")
    quaternion = rot_vis_ned_from_frd.as_quat()
    if not np.all(np.isfinite(quaternion)):
        return state, VisionUpdateResult(False, np.inf, "invalid orientation")

    position_residual = position - state.position_ned
    if max_position_error > 0.0 and np.linalg.norm(position_residual) > max_position_error:
        return state, VisionUpdateResult(False, np.inf, "absolute position gate")
    attitude_residual = (state.rotation_ned_from_frd.inv() * rot_vis_ned_from_frd).as_rotvec()
    residual = np.concatenate([position_residual, attitude_residual])

    observation = np.zeros((6, ERROR_STATE_SIZE), dtype=np.float64)
    observation[0:3, 0:3] = np.eye(3)
    observation[3:6, 6:9] = np.eye(3)
    innovation_covariance = observation @ state.covariance @ observation.T + measurement_covariance
    try:
        solved_residual = np.linalg.solve(innovation_covariance, residual)
    except np.linalg.LinAlgError:
        return state, VisionUpdateResult(False, np.inf, "singular innovation covariance")
    mahalanobis_squared = float(residual @ solved_residual)
    if not np.isfinite(mahalanobis_squared):
        return state, VisionUpdateResult(False, mahalanobis_squared, "non-finite innovation")
    if chi_square_threshold > 0.0 and mahalanobis_squared > chi_square_threshold:
        return state, VisionUpdateResult(False, mahalanobis_squared, "chi-square gate")

    kalman_gain = np.linalg.solve(
        innovation_covariance,
        observation @ state.covariance,
    ).T
    correction = kalman_gain @ residual
    corrected_rotation = state.rotation_ned_from_frd * R.from_rotvec(correction[6:9])

    identity = np.eye(ERROR_STATE_SIZE)
    joseph_left = identity - kalman_gain @ observation
    covariance = (
        joseph_left @ state.covariance @ joseph_left.T
        + kalman_gain @ measurement_covariance @ kalman_gain.T
    )
    reset_jacobian = np.eye(ERROR_STATE_SIZE)
    reset_jacobian[6:9, 6:9] -= 0.5 * _skew(correction[6:9])
    covariance = reset_jacobian @ covariance @ reset_jacobian.T
    covariance = 0.5 * (covariance + covariance.T)

    corrected = ImuEstimatorState(
        position_ned=state.position_ned + correction[0:3],
        velocity_ned=state.velocity_ned + correction[3:6],
        rotation_ned_from_frd=corrected_rotation,
        angular_velocity_frd=state.angular_velocity_frd - correction[12:15],
        accel_bias_frd=state.accel_bias_frd + correction[9:12],
        gyro_bias_frd=state.gyro_bias_frd + correction[12:15],
        covariance=covariance,
        previous_acceleration_frd=(
            None
            if state.previous_acceleration_frd is None
            else state.previous_acceleration_frd.copy()
        ),
        previous_angular_velocity_frd=(
            None
            if state.previous_angular_velocity_frd is None
            else state.previous_angular_velocity_frd.copy()
        ),
    )
    return corrected, VisionUpdateResult(True, mahalanobis_squared, "accepted")


@dataclass(frozen=True)
class _TimedImu:
    timestamp_s: float
    acceleration_frd: np.ndarray
    angular_velocity_frd: np.ndarray


class ErrorStateEstimator:
    """Timestamp-aware ESKF with a bounded rewind/repropagation history."""

    def __init__(
        self,
        *,
        accel_is_specific_force: bool = True,
        velocity_damping_per_s: float = 0.0,
        max_dt_s: float = 0.05,
        accel_noise_std: float = 0.35,
        gyro_noise_std: float = 0.03,
        accel_bias_random_walk_std: float = 0.01,
        gyro_bias_random_walk_std: float = 0.001,
        vision_position_std: float = 0.25,
        vision_orientation_std: float = np.deg2rad(10.0),
        vision_chi_square_threshold: float = 16.812,
        max_position_error: float = 0.0,
        history_duration_s: float = 0.5,
    ) -> None:
        if max_dt_s <= 0.0:
            raise ValueError("max_dt_s must be positive")
        if vision_position_std <= 0.0 or vision_orientation_std <= 0.0:
            raise ValueError("vision standard deviations must be positive")
        process_noises = (
            accel_noise_std,
            gyro_noise_std,
            accel_bias_random_walk_std,
            gyro_bias_random_walk_std,
        )
        if any(noise < 0.0 for noise in process_noises):
            raise ValueError("process noise standard deviations cannot be negative")
        self.accel_is_specific_force = accel_is_specific_force
        self.velocity_damping_per_s = velocity_damping_per_s
        self.max_dt_s = max_dt_s
        self.accel_noise_std = accel_noise_std
        self.gyro_noise_std = gyro_noise_std
        self.accel_bias_random_walk_std = accel_bias_random_walk_std
        self.gyro_bias_random_walk_std = gyro_bias_random_walk_std
        self.vision_position_std = vision_position_std
        self.vision_orientation_std = vision_orientation_std
        self.vision_chi_square_threshold = vision_chi_square_threshold
        self.max_position_error = max_position_error
        self.history_duration_s = max(0.0, history_duration_s)
        self.reset()

    def reset(self, state: ImuEstimatorState | None = None) -> None:
        self.state = ImuEstimatorState.zero() if state is None else state.copy()
        self.last_imu_time_s: float | None = None
        self._imu_history: list[_TimedImu] = []
        self._state_history: list[ImuEstimatorState] = []

    def initialize_stationary(
        self,
        accelerometer_samples: np.ndarray,
        gyroscope_samples: np.ndarray,
        initial_yaw_rad: float = 0.0,
    ) -> bool:
        """Initialize attitude and biases from a stationary, level-ish window."""
        accelerometers = np.asarray(accelerometer_samples, dtype=np.float64)
        gyroscopes = np.asarray(gyroscope_samples, dtype=np.float64)
        if (
            accelerometers.ndim != 2
            or accelerometers.shape[1] != 3
            or gyroscopes.shape != accelerometers.shape
            or len(accelerometers) < 2
        ):
            return False
        mean_accel = np.mean(accelerometers, axis=0)
        mean_gyro = np.mean(gyroscopes, axis=0)
        if not np.all(np.isfinite(mean_accel)) or not np.all(np.isfinite(mean_gyro)):
            return False
        accel_norm = np.linalg.norm(mean_accel)
        if accel_norm < 1e-6:
            return False

        if self.accel_is_specific_force:
            expected_ned = -GRAVITY_NED
            rotation = _shortest_rotation_between(mean_accel, expected_ned)
            # Gravity fixes roll/pitch but not yaw. The race start heading is a
            # permitted static track prior, so apply it about world NED +Z.
            rotation = R.from_euler('z', float(initial_yaw_rad)) * rotation
            expected_body = rotation.inv().apply(expected_ned)
            self.state.rotation_ned_from_frd = rotation
            self.state.accel_bias_frd = mean_accel - expected_body
        else:
            self.state.accel_bias_frd = mean_accel.copy()
        self.state.gyro_bias_frd = mean_gyro
        self.state.angular_velocity_frd = np.zeros(3, dtype=np.float64)
        self.state.previous_acceleration_frd = None
        self.state.previous_angular_velocity_frd = None
        return True

    def process_imu(
        self,
        linear_acceleration_frd: np.ndarray,
        angular_velocity_frd: np.ndarray,
        timestamp_s: float,
    ) -> bool:
        acceleration = np.asarray(linear_acceleration_frd, dtype=np.float64)
        angular_velocity = np.asarray(angular_velocity_frd, dtype=np.float64)
        timestamp = float(timestamp_s)
        if (
            not np.isfinite(timestamp)
            or acceleration.shape != (3,)
            or angular_velocity.shape != (3,)
            or not np.all(np.isfinite(acceleration))
            or not np.all(np.isfinite(angular_velocity))
        ):
            return False
        if self.last_imu_time_s is None:
            self.last_imu_time_s = timestamp
            self.state.previous_acceleration_frd = acceleration.copy()
            self.state.previous_angular_velocity_frd = angular_velocity.copy()
            self.state.angular_velocity_frd = angular_velocity - self.state.gyro_bias_frd
            self._append_history(timestamp, acceleration, angular_velocity)
            return True
        if timestamp <= self.last_imu_time_s:
            return False

        self.state = self._propagate(
            self.state,
            acceleration,
            angular_velocity,
            timestamp - self.last_imu_time_s,
        )
        self.last_imu_time_s = timestamp
        self._append_history(timestamp, acceleration, angular_velocity)
        return True

    def update_vision(
        self,
        position_ned: np.ndarray,
        rotation_ned_from_frd: R,
        covariance: np.ndarray | None = None,
        timestamp_s: float | None = None,
    ) -> VisionUpdateResult:
        measurement_covariance = make_pose_covariance(
            covariance,
            self.vision_position_std,
            self.vision_orientation_std,
        )
        timestamp = None if timestamp_s is None else float(timestamp_s)
        delayed = (
            timestamp is not None
            and self.last_imu_time_s is not None
            and timestamp < self.last_imu_time_s
            and bool(self._state_history)
        )

        if delayed:
            history_times = np.array([item.timestamp_s for item in self._imu_history])
            history_index = int(np.searchsorted(history_times, timestamp, side="right") - 1)
            if history_index < 0:
                return VisionUpdateResult(False, np.inf, "measurement older than history", True)
            update_state = self._state_history[history_index]
        else:
            history_index = len(self._state_history) - 1
            update_state = self.state

        corrected, result = kalman_update_vision_pose(
            update_state,
            position_ned,
            rotation_ned_from_frd,
            measurement_covariance,
            chi_square_threshold=self.vision_chi_square_threshold,
            max_position_error=self.max_position_error,
        )
        applied_timestamp = (
            self._imu_history[history_index].timestamp_s
            if delayed and history_index >= 0
            else timestamp
        )
        result = VisionUpdateResult(
            result.accepted,
            result.mahalanobis_squared,
            result.reason,
            delayed,
            applied_timestamp,
        )
        if not result.accepted:
            return result

        if delayed:
            self._state_history[history_index] = corrected.copy()
            replayed = corrected
            for index in range(history_index + 1, len(self._imu_history)):
                previous_time = self._imu_history[index - 1].timestamp_s
                sample = self._imu_history[index]
                replayed = self._propagate(
                    replayed,
                    sample.acceleration_frd,
                    sample.angular_velocity_frd,
                    sample.timestamp_s - previous_time,
                )
                self._state_history[index] = replayed.copy()
            self.state = replayed
        else:
            self.state = corrected
            if history_index >= 0:
                self._state_history[history_index] = corrected.copy()
        return result

    def _propagate(
        self,
        state: ImuEstimatorState,
        acceleration: np.ndarray,
        angular_velocity: np.ndarray,
        dt_s: float,
    ) -> ImuEstimatorState:
        return integrate_imu_state(
            state,
            acceleration,
            angular_velocity,
            dt_s,
            accel_is_specific_force=self.accel_is_specific_force,
            velocity_damping_per_s=self.velocity_damping_per_s,
            max_dt_s=self.max_dt_s,
            accel_noise_std=self.accel_noise_std,
            gyro_noise_std=self.gyro_noise_std,
            accel_bias_random_walk_std=self.accel_bias_random_walk_std,
            gyro_bias_random_walk_std=self.gyro_bias_random_walk_std,
        )

    def _append_history(
        self,
        timestamp: float,
        acceleration: np.ndarray,
        angular_velocity: np.ndarray,
    ) -> None:
        self._imu_history.append(
            _TimedImu(timestamp, acceleration.copy(), angular_velocity.copy())
        )
        self._state_history.append(self.state.copy())
        oldest_time = timestamp - self.history_duration_s
        while len(self._imu_history) > 1 and self._imu_history[1].timestamp_s < oldest_time:
            self._imu_history.pop(0)
            self._state_history.pop(0)


def fuse_vision_pose(
    state: ImuEstimatorState,
    pos_vis_ned: np.ndarray,
    rot_vis_ned_from_frd: R,
    dt_vis_s: float | None = None,
    *,
    alpha_pos: float = 0.3,
    alpha_rot: float = 0.2,
    alpha_vel: float = 0.2,
    max_pos_err: float = 10.0,
) -> ImuEstimatorState:
    """Deprecated fixed-gain compatibility helper; runtime code uses the ESKF."""
    pos_vis = np.asarray(pos_vis_ned, dtype=np.float64)
    pos_err = np.linalg.norm(pos_vis - state.position_ned)
    if pos_err > max_pos_err and np.linalg.norm(state.position_ned) > 0.1:
        return state
    next_pos = (1.0 - alpha_pos) * state.position_ned + alpha_pos * pos_vis
    next_vel = state.velocity_ned.copy()
    if dt_vis_s is not None and 0.005 < dt_vis_s < 1.0:
        vel_vis = (pos_vis - state.position_ned) / dt_vis_s
        next_vel = (1.0 - alpha_vel) * state.velocity_ned + alpha_vel * vel_vis
    key_rots = R.from_quat([state.rotation_ned_from_frd.as_quat(), rot_vis_ned_from_frd.as_quat()])
    next_rot = Slerp([0.0, 1.0], key_rots)(alpha_rot)
    result = state.copy()
    result.position_ned = next_pos
    result.velocity_ned = next_vel
    result.rotation_ned_from_frd = next_rot
    return result
