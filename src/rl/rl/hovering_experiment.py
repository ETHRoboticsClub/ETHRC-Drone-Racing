"""Observation builder for the hovering / gate-tracking RL policy."""

from __future__ import annotations

import numpy as np
import torch
from scipy.spatial.transform import Rotation as R

from policy_model import PolicyModel


# Match the values from the Isaac Lab training env.
_REL_POSITION_DIVISOR = 10.0
_ANG_COMMAND_SCALE = 10.0
_GRAVITY_WORLD = np.array([0.0, 0.0, -9.81], dtype=float)

OBS_DIM = 12
STATE_DIM = 13

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
        self.load_policy(model_path, model_type)

    def load_policy(self, model_path: str, model_type: str) -> None:
        self.policy = PolicyModel(model_path=model_path, model_type=model_type)

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
        omega_est[2] *= -1.0
        return omega_est.astype(np.float32)

    def _obs_base_lin_vel(self, x_current: np.ndarray) -> np.ndarray:
        """
        [3] linear velocity in simulation body frame [m/s].

        ``x[3:6]`` from ``state_bridge_node`` is world-frame (Vicon derivative).
        Rotated with the same sim body attitude as gravity / waypoint obs.
        """
        s = np.asarray(x_current, dtype=float).reshape(STATE_DIM)
        vel_w = s[3:6]
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
        return np.concatenate(
            [
                self._obs_projected_gravity_body(x_current),
                self._obs_base_ang_vel(x_current),
                self._obs_waypoint_forward_body(x_current, goal_position),
                self._obs_base_lin_vel(x_current),
            ],
            dtype=np.float32,
        )
    
    def compute_actions(self, observation: np.ndarray) -> np.ndarray:
        """
        Policy inference → drone-frame rate commands for Betaflight / MSP.
        """
        if self.policy is None:
            raise RuntimeError("Policy not loaded. Call load_policy() first.")

        obs_t = torch.tensor(observation, dtype=torch.float32).unsqueeze(0)
        raw = self.policy.inference(obs_t).squeeze(0).cpu().numpy().clip(-1.0, 1.0)

        # [-1, 1] -> [0, 1] with 0.7 safety cap
        thrust_cmd = min(float((raw[0] + 1.0) * 0.5), 0.7)

        # Policy outputs rate commands in sim FLU frame.
        # MAVSDK AttitudeRate / Betaflight MSP use FRD: X (roll) is shared,
        # negate Y (pitch) and Z (yaw).
        omega_cmd_FLU_rad = np.asarray(raw[1:4], dtype=float) * _ANG_COMMAND_SCALE
        
        # FLU -> FRD/NED:
        omega_cmd_FRD_rad = omega_cmd_FLU_rad.copy()
        omega_cmd_FRD_rad[1] *= -1.0
        omega_cmd_FRD_rad[2] *= -1.0

        return np.array(
            [
                float(np.degrees(omega_cmd_FRD_rad[0])),
                float(np.degrees(omega_cmd_FRD_rad[1])),
                float(np.degrees(omega_cmd_FRD_rad[2])),
                thrust_cmd,
            ],
            dtype=np.float32,
        )

    def predict_commands(
        self, x_current: np.ndarray, goal_state: np.ndarray
    ) -> np.ndarray:
        """``build_observation`` → ``compute_actions``."""
        observation = self.build_observation(x_current, goal_state)
        actions = self.compute_actions(observation)
        return actions
