"""Ground-truth VQ1 gate-centering controller for FlightSim's velocity API."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation as R

from rl.aigp_policy_runtime import FRAME_FLIP, Gate, ned_frd_state_to_training


@dataclass(frozen=True)
class VelocityFieldConfig:
    target_speed_m_s: float = 1.0
    position_gain_per_s: float = 0.3
    lateral_speed_limit_m_s: float = 0.5
    max_command_speed_m_s: float = 1.2


@dataclass(frozen=True)
class VelocityFieldTelemetry:
    forward_distance_m: float
    centering_error_m: float
    command_speed_m_s: float


@dataclass(frozen=True)
class RateControllerConfig(VelocityFieldConfig):
    velocity_gain_per_s: float = 0.8
    vertical_position_gain_per_s: float = 1.0
    vertical_speed_limit_m_s: float = 0.5
    vertical_velocity_gain_per_s: float = 2.0
    acceleration_limit_m_s2: float = 0.5
    attitude_gain_per_s: float = 1.0
    body_rate_limit_rad_s: float = 0.5
    roll_attitude_sign: float = -1.0
    roll_rate_sign: float = 1.0
    pitch_rate_sign: float = -1.0
    gravity_m_s2: float = 9.80665
    thrust_fit_intercept_ned_m_s2: float = 8.419816438065174
    thrust_fit_slope_ned_m_s2: float = -30.33559184948186
    thrust_calibration_body_up: float = 0.9521197
    minimum_thrust: float = 0.15
    maximum_thrust: float = 0.45


@dataclass(frozen=True)
class RateControllerTelemetry(VelocityFieldTelemetry):
    desired_acceleration_nwu: np.ndarray
    desired_body_rate_frd: np.ndarray
    desired_euler_ned_frd: np.ndarray
    current_euler_ned_frd: np.ndarray
    thrust_command: float


def _limit_norm(vector: np.ndarray, maximum: float) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    if norm <= maximum or norm <= 1.0e-9:
        return vector
    return vector * (float(maximum) / norm)


def gate_forward_nwu(gate: Gate) -> np.ndarray:
    """Return the gate-local +Y passage axis in the NWU world frame."""

    forward = np.asarray(gate.rotation.apply([0.0, 1.0, 0.0]), dtype=np.float64)
    norm = float(np.linalg.norm(forward))
    if not np.isfinite(norm) or norm < 1.0e-9:
        raise ValueError("Gate forward axis is invalid")
    return forward / norm


def velocity_field_command_ned(
    state_ned_frd: np.ndarray,
    gate: Gate,
    config: VelocityFieldConfig,
) -> tuple[np.ndarray, VelocityFieldTelemetry]:
    """Compute a stabilized LOCAL_NED velocity command for one AIGP gate.

    This is the translational portion of the lap-capable Isaac controller from
    commit c786ce4. Constant passage-axis velocity prevents braking at the gate
    plane, while the orthogonal velocity independently converges to the gate
    centre. FlightSim closes the unknown vehicle dynamics internally.
    """

    state = np.asarray(state_ned_frd, dtype=np.float64)
    if state.shape != (13,) or not np.all(np.isfinite(state)):
        raise ValueError("VQ1 state must be a finite 13-vector")
    quaternion_norm = float(np.linalg.norm(state[6:10]))
    if not 0.5 <= quaternion_norm <= 1.5:
        raise ValueError(f"VQ1 quaternion norm is invalid: {quaternion_norm}")

    training_state = ned_frd_state_to_training(state, velocity_frame="world_ned")
    forward_nwu = gate_forward_nwu(gate)
    offset_nwu = np.asarray(gate.position, dtype=np.float64) - training_state.position_w
    forward_distance = float(np.dot(offset_nwu, forward_nwu))
    centering_error = offset_nwu - forward_distance * forward_nwu
    centering_velocity = _limit_norm(
        float(config.position_gain_per_s) * centering_error,
        float(config.lateral_speed_limit_m_s),
    )
    command_nwu = float(config.target_speed_m_s) * forward_nwu + centering_velocity
    command_nwu = _limit_norm(command_nwu, float(config.max_command_speed_m_s))
    command_ned = FRAME_FLIP * command_nwu

    telemetry = VelocityFieldTelemetry(
        forward_distance_m=forward_distance,
        centering_error_m=float(np.linalg.norm(centering_error)),
        command_speed_m_s=float(np.linalg.norm(command_ned)),
    )
    return command_ned, telemetry


def _velocity_field_nwu(
    position_nwu: np.ndarray,
    gate: Gate,
    config: VelocityFieldConfig,
) -> tuple[np.ndarray, float, float]:
    forward_nwu = gate_forward_nwu(gate)
    offset_nwu = np.asarray(gate.position, dtype=np.float64) - position_nwu
    forward_distance = float(np.dot(offset_nwu, forward_nwu))
    centering_error = offset_nwu - forward_distance * forward_nwu
    centering_velocity = _limit_norm(
        float(config.position_gain_per_s) * centering_error,
        float(config.lateral_speed_limit_m_s),
    )
    command_nwu = float(config.target_speed_m_s) * forward_nwu + centering_velocity
    command_nwu = _limit_norm(command_nwu, float(config.max_command_speed_m_s))
    return command_nwu, forward_distance, float(np.linalg.norm(centering_error))


def rate_thrust_command_frd(
    state_ned_frd: np.ndarray,
    gate: Gate,
    config: RateControllerConfig,
    acceleration_override_nwu: np.ndarray | None = None,
) -> tuple[np.ndarray, RateControllerTelemetry]:
    """Compute ``[roll, pitch, yaw deg/s, collective]`` for FlightSim V4."""

    state = np.asarray(state_ned_frd, dtype=np.float64)
    if state.shape != (13,) or not np.all(np.isfinite(state)):
        raise ValueError("VQ1 state must be a finite 13-vector")
    training_state = ned_frd_state_to_training(state, velocity_frame="world_ned")
    desired_velocity_nwu, forward_distance, center_error = _velocity_field_nwu(
        training_state.position_w, gate, config
    )
    desired_velocity_nwu[2] = float(
        np.clip(
            float(config.vertical_position_gain_per_s)
            * (float(gate.position[2]) - float(training_state.position_w[2])),
            -float(config.vertical_speed_limit_m_s),
            float(config.vertical_speed_limit_m_s),
        )
    )
    if acceleration_override_nwu is None:
        velocity_error = desired_velocity_nwu - FRAME_FLIP * state[3:6]
        velocity_error[2] *= float(config.vertical_velocity_gain_per_s) / float(
            config.velocity_gain_per_s
        )
        acceleration_nwu = _limit_norm(
            float(config.velocity_gain_per_s) * velocity_error,
            float(config.acceleration_limit_m_s2),
        )
    else:
        acceleration_nwu = np.asarray(acceleration_override_nwu, dtype=np.float64)
        if acceleration_nwu.shape != (3,) or not np.all(np.isfinite(acceleration_nwu)):
            raise ValueError("Acceleration override must be a finite 3-vector")

    specific_thrust_nwu = acceleration_nwu + np.array(
        [0.0, 0.0, float(config.gravity_m_s2)], dtype=np.float64
    )
    desired_z = specific_thrust_nwu / max(np.linalg.norm(specific_thrust_nwu), 1.0e-9)
    heading = gate_forward_nwu(gate)
    desired_x = heading - np.dot(heading, desired_z) * desired_z
    desired_x /= max(np.linalg.norm(desired_x), 1.0e-9)
    desired_y = np.cross(desired_z, desired_x)
    desired_y /= max(np.linalg.norm(desired_y), 1.0e-9)
    desired_x = np.cross(desired_y, desired_z)
    desired_rotation = np.column_stack((desired_x, desired_y, desired_z))

    rotation = training_state.rotation_wb.as_matrix()
    # Convert both attitudes to the telemetry's explicit NED/FRD Euler
    # convention. This avoids a frame ambiguity exposed by live lateral
    # braking: build 3390's physical-rate extension uses the opposite sign for
    # both roll and pitch relative to its ground-truth Euler convention.
    flip_matrix = np.diag(FRAME_FLIP)
    desired_rotation_ned_frd = flip_matrix @ desired_rotation @ flip_matrix
    current_rotation_ned_frd = R.from_quat(
        [state[7], state[8], state[9], state[6]]
    ).as_matrix()
    desired_euler = R.from_matrix(desired_rotation_ned_frd).as_euler("xyz")
    # ODOMETRY's reported roll is consistent with commanded roll rate, but the
    # simulator's lateral thrust response is opposite the nominal FRD attitude
    # geometry. Reflect only the roll target; keep feedback in the measured
    # Euler convention.
    desired_euler[0] *= float(config.roll_attitude_sign)
    current_euler = R.from_matrix(current_rotation_ned_frd).as_euler("xyz")
    euler_error = (desired_euler - current_euler + np.pi) % (2.0 * np.pi) - np.pi
    desired_rate_frd = float(config.attitude_gain_per_s) * np.array(
        [
            float(config.roll_rate_sign) * euler_error[0],
            float(config.pitch_rate_sign) * euler_error[1],
            euler_error[2],
        ],
        dtype=np.float64,
    )
    desired_rate_frd = _limit_norm(
        desired_rate_frd, float(config.body_rate_limit_rad_s)
    )

    current_body_z_nwu = rotation[:, 2]
    collective_acceleration = max(float(np.dot(specific_thrust_nwu, current_body_z_nwu)), 0.0)
    slope = float(config.thrust_fit_slope_ned_m_s2)
    if not slope < -1.0e-6:
        raise ValueError(f"Thrust calibration slope must be negative, got {slope}")
    # The affine calibration is az_NED=b+s*u. Rewriting around physical
    # collective acceleration q gives u=(q-g+b)/(-s), preserving the measured
    # hover value -b/s while compensating modest vehicle tilt.
    thrust = (
        collective_acceleration
        - float(config.gravity_m_s2)
        + float(config.thrust_fit_intercept_ned_m_s2)
    ) / (-slope)
    current_body_up = max(float(current_body_z_nwu[2]), 0.5)
    thrust *= float(config.thrust_calibration_body_up) / current_body_up
    thrust = float(np.clip(thrust, config.minimum_thrust, config.maximum_thrust))

    command = np.concatenate((np.degrees(desired_rate_frd), [thrust]))
    telemetry = RateControllerTelemetry(
        forward_distance_m=forward_distance,
        centering_error_m=center_error,
        command_speed_m_s=float(np.linalg.norm(desired_velocity_nwu)),
        desired_acceleration_nwu=acceleration_nwu,
        desired_body_rate_frd=desired_rate_frd,
        desired_euler_ned_frd=desired_euler,
        current_euler_ned_frd=current_euler,
        thrust_command=thrust,
    )
    return command, telemetry


def vehicle_tilt_rad(state_ned_frd: np.ndarray) -> float:
    """Return tilt of body-up from world-up, independent of yaw."""

    training_state = ned_frd_state_to_training(
        np.asarray(state_ned_frd, dtype=np.float64), velocity_frame="world_ned"
    )
    body_up_nwu = training_state.rotation_wb.apply([0.0, 0.0, 1.0])
    return float(np.arccos(np.clip(body_up_nwu[2], -1.0, 1.0)))
