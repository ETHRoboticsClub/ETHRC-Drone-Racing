"""Observation builder for the hovering / gate-tracking RL policy."""

from __future__ import annotations

import os
import math

import numpy as np
import torch
from scipy.spatial.transform import Rotation as R

from rl.policy_model import PolicyModel


# Match the values from the Isaac Lab training env.
_REL_POSITION_DIVISOR = 10.0
# The real Boreas ACTUAL rate profile is linear with 500 deg/s at full stick.
RATE_CMD_LIMIT_DEG_S = float(os.environ.get("RL_RATE_LIMIT_DEG_S", "500.0"))
ANG_COMMAND_SCALE = float(
    os.environ.get("RL_ANG_COMMAND_SCALE", str(math.radians(500.0)))
)

# The Boreas residual environment uses the measured 916 gf/prop endpoint.
# Deployment and SITL pass through Betaflight, so convert the desired force
# fraction back to a motor-speed command. The endpoint is measured; the
# quadratic curve remains a documented prior until a full thrust sweep exists.
TRAINING_MAX_THRUST_N = float(
    os.environ.get("RL_TRAINING_MAX_THRUST_N", "35.9315656")
)
BOREAS_MAX_THRUST_N = float(os.environ.get("RL_BOREAS_MAX_THRUST_N", "35.9315656"))
BOREAS_THRUST_TO_WEIGHT = float(
    os.environ.get("RL_BOREAS_THRUST_TO_WEIGHT", "4.2")
)
THRUST_CMD_LIMIT = float(os.environ.get("RL_THRUST_LIMIT", "1.0"))
_GRAVITY_WORLD = np.array([0.0, 0.0, -9.81], dtype=float)
_RESIDUAL_POLICY = os.environ.get("RL_RESIDUAL_POLICY", "true").lower() in (
    "1", "true", "yes", "on"
)
# Safe deployment default: run the Gazebo-qualified nominal controller. A
# learned residual is opt-in until its exact checkpoint passes both the paired
# Isaac baseline gate and the armed Betaflight/Gazebo gate.
_RESIDUAL_SCALE = float(os.environ.get("RL_RESIDUAL_SCALE", "0.0"))
_RESIDUAL_MASK = np.fromstring(
    os.environ.get("RL_RESIDUAL_MASK", "0,1,1,0"), sep=",", dtype=float
)
if _RESIDUAL_MASK.shape != (4,) or not np.all(np.isin(_RESIDUAL_MASK, (0.0, 1.0))):
    raise ValueError(
        "RL_RESIDUAL_MASK must contain four comma-separated 0/1 values "
        "for collective, roll, pitch, yaw"
    )
_OUTER_POSITION_GAIN_XY = float(os.environ.get("RL_OUTER_POSITION_GAIN_XY", "1.5"))
_OUTER_VELOCITY_GAIN_XY = float(os.environ.get("RL_OUTER_VELOCITY_GAIN_XY", "2.0"))
_OUTER_ATTITUDE_BANDWIDTH = float(os.environ.get("RL_OUTER_ATTITUDE_BANDWIDTH", "4.0"))
_OUTER_POSITION_GAIN_Z = float(os.environ.get("RL_OUTER_POSITION_GAIN_Z", "2.0"))
_OUTER_VELOCITY_GAIN_Z = float(os.environ.get("RL_OUTER_VELOCITY_GAIN_Z", "3.0"))

OBS_DIM = 12
STATE_DIM = 13


def training_thrust_to_betaflight_command(raw_thrust_action: float) -> float:
    """Map the Isaac linear-force action to the measured Boreas motor curve."""
    action = float(np.clip(raw_thrust_action, -1.0, 1.0))
    training_fraction = (action + 1.0) * 0.5
    desired_thrust_n = TRAINING_MAX_THRUST_N * training_fraction
    command = math.sqrt(
        max(0.0, min(desired_thrust_n / BOREAS_MAX_THRUST_N, 1.0))
    )
    return min(command, THRUST_CMD_LIMIT)


def policy_rates_to_betaflight_deg_s(raw_rates: np.ndarray) -> np.ndarray:
    """Map Isaac FLU rate actions to the real Boreas Betaflight RC contract."""
    omega_flu_rad_s = np.asarray(raw_rates, dtype=float) * ANG_COMMAND_SCALE
    omega_betaflight_rad_s = omega_flu_rad_s * np.array([1.0, 1.0, -1.0])
    command = np.degrees(omega_betaflight_rad_s)
    return np.clip(command, -RATE_CMD_LIMIT_DEG_S, RATE_CMD_LIMIT_DEG_S)


class HoveringExperiment:
    """
    State: ``[
      p(3),
      v(3),
      q_xyzw(4),
      omega(3)
    ]`` from ``drone_state_sim`` (quat at 6:10).

    ``build_observation`` layout (12-dim, must match ``policy.pt``):
      [0:3] projected gravity,
      [3:6] base_ang_vel
      [6:9] next waypoint rel
      [9:12] base_lin_vel
    """

    def __init__(self, model_path: str, model_type: str = "pytorch") -> None:
        self._model_path = model_path
        self._model_type = model_type
        self.policy: PolicyModel | None = None
        self.last_observation: np.ndarray | None = None
        self.load_policy(model_path, model_type)

    def load_policy(self, model_path: str, model_type: str) -> None:
        bounded = os.environ.get("RL_POLICY_OUTPUT_TANH", "false").lower() in (
            "1", "true", "yes", "on"
        )
        self.policy = PolicyModel(
            model_path=model_path,
            model_type=model_type,
            output_activation="tanh" if bounded else None,
        )

    @staticmethod
    def _body_rot_world_to_sim(state: np.ndarray) -> R:
        """World → simulation body (est quaternion + ``_R_EST_TO_SIM`` at top of file)."""
        s = np.asarray(state, dtype=float).reshape(STATE_DIM)
        quat_est_xyzw = np.array([s[6], s[7], s[8], s[9]], dtype=float)
        rotation = R.from_quat(quat_est_xyzw)
        return rotation

    def _obs_projected_gravity_body(self, x_current: np.ndarray) -> np.ndarray:
        """[3] unit gravity direction in simulation body frame."""
        g_body = self._body_rot_world_to_sim(x_current).inv().apply(_GRAVITY_WORLD)
        g_norm = float(np.linalg.norm(g_body))
        if g_norm < 1e-9:
            return np.array([0.0, 0.0, -1.0], dtype=np.float32)
        return (g_body / g_norm).astype(np.float32)

    def _obs_base_ang_vel(self, x_current: np.ndarray) -> np.ndarray:
        """[3] angular velocity in simulation body frame [rad/s] (IMU ω + axis fix)."""
        s = np.asarray(x_current, dtype=float).reshape(STATE_DIM)
        omega_est = s[10:13].copy()
        return omega_est.astype(np.float32)

    def _obs_base_lin_vel(self, x_current: np.ndarray) -> np.ndarray:
        """
        [3] linear velocity in simulation body frame [m/s].

        ``x[3:6]`` from ``state_bridge_node`` is world-frame (Vicon derivative).
        Rotated with the same sim body attitude as gravity / waypoint obs.
        """
        s = np.asarray(x_current, dtype=float).reshape(STATE_DIM)
        vel_w = np.array([s[3], s[4], s[5]], dtype=float)
        vel_b = self._body_rot_world_to_sim(x_current).inv().apply(vel_w.astype(np.float32)).astype(np.float32)
        return vel_b

    def _obs_waypoint_forward_body(
        self,
        x_current: np.ndarray,
        goal_position: np.ndarray,
    ) -> np.ndarray:
        """[3] waypoint expressed in FLU body frame."""
        drone_pos_w = x_current[:3]
        offset_w = goal_position[:3] - drone_pos_w
        offset_b = self._body_rot_world_to_sim(x_current).inv().apply(offset_w.astype(np.float32)).astype(np.float32)
        offset_b_scaled = offset_b / _REL_POSITION_DIVISOR
        return offset_b_scaled.astype(np.float32)



    def build_observation(
        self,
        x_current: np.ndarray,
        goal_position: np.ndarray,
    ) -> np.ndarray:
        obs = np.concatenate(
            [
                self._obs_projected_gravity_body(x_current),
                self._obs_base_ang_vel(x_current),
                self._obs_waypoint_forward_body(x_current, goal_position),
                self._obs_base_lin_vel(x_current),
            ],
            dtype=np.float32,
        )
        self.last_observation = obs.copy()
        return obs
    
    def compute_actions(self, observation: np.ndarray) -> np.ndarray:
        """
        Policy inference → drone-frame rate commands for Betaflight / MSP.
        """
        if _RESIDUAL_POLICY and _RESIDUAL_SCALE == 0.0:
            # Do not even evaluate an unqualified checkpoint in safe nominal
            # mode. Besides saving inference work, this prevents malformed or
            # non-finite policy output from contaminating ``0 * residual``.
            raw = self._outer_loop_action(observation)
        else:
            if self.policy is None:
                raise RuntimeError("Policy not loaded. Call load_policy() first.")
            obs_t = torch.tensor(observation, dtype=torch.float32).unsqueeze(0)
            raw = (
                self.policy.inference(obs_t)
                .squeeze(0)
                .cpu()
                .numpy()
                .clip(-1.0, 1.0)
            )
        if _RESIDUAL_POLICY and _RESIDUAL_SCALE != 0.0:
            raw = np.clip(
                self._outer_loop_action(observation)
                + _RESIDUAL_SCALE * _RESIDUAL_MASK * raw,
                -1.0,
                1.0,
            )

        thrust_cmd = training_thrust_to_betaflight_command(raw[0])

        # The policy uses ROS FLU angular-rate signs. Betaflight's RC rate
        # interface uses stick signs: positive pitch means nose-down, which is
        # already positive FLU pitch. Positive yaw is clockwise, so only yaw
        # changes sign. This is not a generic FLU -> FRD vector transform.
        cmd_deg_s = policy_rates_to_betaflight_deg_s(raw[1:4])

        return np.array(
            [
                float(cmd_deg_s[0]),
                float(cmd_deg_s[1]),
                float(cmd_deg_s[2]),
                thrust_cmd,
            ],
            dtype=np.float32,
        )

    @staticmethod
    def _outer_loop_action(observation: np.ndarray) -> np.ndarray:
        """Conservative nominal action used by residual-RL policies."""
        obs = np.asarray(observation, dtype=float).reshape(OBS_DIM)
        gravity_b = obs[0:3]
        offset_b = obs[6:9] * _REL_POSITION_DIVISOR
        velocity_b = obs[9:12]

        accel_x = _OUTER_POSITION_GAIN_XY * offset_b[0] - _OUTER_VELOCITY_GAIN_XY * velocity_b[0]
        accel_y = _OUTER_POSITION_GAIN_XY * offset_b[1] - _OUTER_VELOCITY_GAIN_XY * velocity_b[1]
        desired_pitch = float(np.clip(accel_x / 9.81, -0.35, 0.35))
        desired_roll = float(np.clip(-accel_y / 9.81, -0.35, 0.35))
        roll = math.atan2(-gravity_b[1], -gravity_b[2])
        pitch = math.atan2(
            gravity_b[0], math.hypot(gravity_b[1], gravity_b[2])
        )

        target = np.zeros(4, dtype=np.float32)
        target[1] = _OUTER_ATTITUDE_BANDWIDTH * (desired_roll - roll) / ANG_COMMAND_SCALE
        target[2] = _OUTER_ATTITUDE_BANDWIDTH * (desired_pitch - pitch) / ANG_COMMAND_SCALE

        upright = max(-gravity_b[2], 0.5)
        accel_z = (
            _OUTER_POSITION_GAIN_Z * offset_b[2]
            - _OUTER_VELOCITY_GAIN_Z * velocity_b[2]
        )
        thrust_fraction = np.clip(
            (1.0 + accel_z / 9.81) / (BOREAS_THRUST_TO_WEIGHT * upright),
            0.0,
            1.0,
        )
        target[0] = 2.0 * thrust_fraction - 1.0
        return target

    def predict_commands(
        self, x_current: np.ndarray, goal_state: np.ndarray
    ) -> np.ndarray:
        """``build_observation`` → ``compute_actions``."""
        observation = self.build_observation(x_current, goal_state)
        actions = self.compute_actions(observation)
        return actions
