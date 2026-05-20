"""Observation builder for the hovering / gate-tracking RL policy."""

from __future__ import annotations

import os

import numpy as np
import torch
from scipy.spatial.transform import Rotation as R

from rl.policy_model import PolicyModel

# ---------------------------------------------------------------------------
# Estimation body → simulation body. Pick ONE line (comment out the others).
# IRL / BF / Vicon body: FRD (X fwd, Y right, Z down).
# Isaac training body: FLU (X fwd, Y left, Z up) — flip Y and Z via 180° roll.
# ---------------------------------------------------------------------------
# _R_EST_TO_SIM = R.identity()
_R_EST_TO_SIM = R.from_euler("x", np.pi)  # FRD (Z down) → FLU (Z up)
# _R_EST_TO_SIM = R.from_euler("xyz", np.deg2rad([0.0, 0.0, -90.0]))  # yaw only: Y-fwd → X-fwd, does NOT fix Z

# Match Isaac ``next_waypoints_rel_body`` / ``_REL_POSITION_SCALE`` from training env.
_REL_POSITION_SCALE = 10.0
_ANG_COMMAND_SCALE = 10.0
_RAD_TO_DEG = 180.0 / np.pi
_POLICY_FILENAME = "policy.pt"


class HoveringExperiment:
    """
    State: ``[p(3), v(3), q_xyzw(4), omega(3)]`` from ``drone_state_sim`` (quat at 6:10).

    ``build_observation`` layout (15-dim, must match ``policy.pt``):
      [0:3] base_lin_vel, [3:6] base_ang_vel, [6:9] projected gravity,
      [9:12] next waypoint rel, [12:15] gate/goal forward (+Y) in body.
    """

    OBS_DIM = 15
    STATE_DIM = 13
    GRAVITY_WORLD = np.array([0.0, 0.0, -9.81], dtype=float)
    GATE_FORWARD_LOCAL = np.array([0.0, 1.0, 0.0], dtype=float)  # gate +Y in goal frame

    def __init__(self, model_path: str | None = None) -> None:
        self._model_path = model_path
        self.policy: PolicyModel | None = None

    @staticmethod
    def default_policy_path() -> str:
        from ament_index_python.packages import get_package_share_directory

        share = get_package_share_directory("rl")
        return os.path.join(share, "models", _POLICY_FILENAME)

    def load_policy(self, model_path: str | None = None) -> None:
        path = model_path or self._model_path or self.default_policy_path()
        self._model_path = path
        self.policy = PolicyModel(path)

    @staticmethod
    def _body_rot_world_to_sim(state: np.ndarray) -> R:
        """World → simulation body (est quaternion + ``_R_EST_TO_SIM`` at top of file)."""
        s = np.asarray(state, dtype=float).reshape(HoveringExperiment.STATE_DIM)
        quat_est_xyzw = np.array([s[6], s[7], s[8], s[9]], dtype=float)
        return _R_EST_TO_SIM * R.from_quat(quat_est_xyzw)

    def _obs_projected_gravity_body(self, x_current: np.ndarray) -> np.ndarray:
        """[3] unit gravity direction in simulation body frame."""
        g_body = self._body_rot_world_to_sim(x_current).inv().apply(self.GRAVITY_WORLD)
        g_norm = float(np.linalg.norm(g_body))
        if g_norm < 1e-9:
            return np.array([0.0, 0.0, -1.0], dtype=np.float32)
        return (g_body / g_norm).astype(np.float32)

    def _obs_base_ang_vel(self, x_current: np.ndarray) -> np.ndarray:
        """[3] angular velocity in simulation body frame [rad/s] (IMU ω + axis fix)."""
        s = np.asarray(x_current, dtype=float).reshape(self.STATE_DIM)
        omega_est = s[10:13]
        return _R_EST_TO_SIM.apply(omega_est).astype(np.float32)

    def _obs_base_lin_vel(self, x_current: np.ndarray) -> np.ndarray:
        """
        [3] linear velocity in simulation body frame [m/s].

        ``x[3:6]`` from ``state_bridge_node`` is world-frame (Vicon derivative).
        Rotated with the same sim body attitude as gravity / waypoint obs.
        """
        s = np.asarray(x_current, dtype=float).reshape(self.STATE_DIM)
        vel_w = s[3:6]
        return self._body_rot_world_to_sim(x_current).inv().apply(vel_w).astype(np.float32)

    def _obs_next_waypoint_rel_body(
        self,
        x_current: np.ndarray,
        goal_state: np.ndarray,
    ) -> np.ndarray:
        """
        [3] goal relative to drone in simulation body axes (k=1), scaled.

        ``offset_b[0]`` = forward (X), ``offset_b[1]`` = left (Y), ``offset_b[2]`` = up (Z)
        distance to goal along each body axis (not just Euclidean range).

        Same as Isaac ``next_waypoints_rel_body`` with ``num_waypoints=1``:
          offset_w = goal_pos_w - drone_pos_w
          offset_b = quat_apply_inverse(drone_quat_w, offset_w) / _REL_POSITION_SCALE
        """
        x = np.asarray(x_current, dtype=float).reshape(self.STATE_DIM)
        goal = np.asarray(goal_state, dtype=float).reshape(self.STATE_DIM)

        offset_w = goal[0:3] - x[0:3]
        offset_b = self._body_rot_world_to_sim(x).inv().apply(offset_w) / _REL_POSITION_SCALE
        return offset_b.astype(np.float32)

    def _obs_gate_forward_body(
        self,
        x_current: np.ndarray,
        goal_state: np.ndarray,
    ) -> np.ndarray:
        """[3] goal/gate +Y axis expressed in simulation body frame."""
        goal = np.asarray(goal_state, dtype=float).reshape(self.STATE_DIM)
        # ``drone_controller_node`` stores goal quaternion as w,x,y,z at indices 6:10.
        quat_goal_xyzw = np.array([goal[7], goal[8], goal[9], goal[6]], dtype=float)
        forward_w = R.from_quat(quat_goal_xyzw).apply(self.GATE_FORWARD_LOCAL)
        forward_b = self._body_rot_world_to_sim(x_current).inv().apply(forward_w)
        norm = float(np.linalg.norm(forward_b))
        if norm < 1e-9:
            return np.array([0.0, 1.0, 0.0], dtype=np.float32)
        return (forward_b / norm).astype(np.float32)

    def build_observation(
        self,
        x_current: np.ndarray,
        goal_state: np.ndarray,
    ) -> np.ndarray:
        return np.concatenate(
            [
                self._obs_base_lin_vel(x_current),
                self._obs_base_ang_vel(x_current),
                self._obs_projected_gravity_body(x_current),
                self._obs_next_waypoint_rel_body(x_current, goal_state),
                self._obs_gate_forward_body(x_current, goal_state),
            ],
            dtype=np.float32,
        )
    
    def compute_actions(self, observation: np.ndarray) -> np.ndarray:
        """
        Policy inference → drone-frame rate commands for Betaflight / MSP.

        Policy outputs body rates in **simulation** frame (FLU: X fwd, Z up).
        Rotated to **estimation / drone** body with ``_R_EST_TO_SIM.inv()`` (same
        inverse as obs: sim → drone). Thrust is unchanged (scalar).
        """
        if self.policy is None:
            raise RuntimeError("Policy not loaded. Call load_policy() first.")

        obs_t = torch.tensor(observation, dtype=torch.float32).unsqueeze(0)
        raw = self.policy.inference(obs_t).squeeze(0).cpu().numpy().clip(-1.0, 1.0)

        thrust_cmd = min(float((raw[0] + 1.0) * 0.5), 0.7)  # safety cap

        # Sim-frame angular rate command [rad/s] from policy axes (X, Y, Z).
        omega_cmd_sim_rad = np.asarray(raw[1:4], dtype=float) * _ANG_COMMAND_SCALE
        omega_cmd_drone_rad = _R_EST_TO_SIM.inv().apply(omega_cmd_sim_rad)

        return np.array(
            [
                thrust_cmd,
                float(np.degrees(omega_cmd_drone_rad[0])),
                float(np.degrees(omega_cmd_drone_rad[1])),
                float(np.degrees(omega_cmd_drone_rad[2])),
            ],
            dtype=np.float32,
        )

    def predict_commands(
        self, x_current: np.ndarray, goal_state: np.ndarray
    ) -> np.ndarray:
        """``build_observation`` → ``compute_actions``."""
        observation = self.build_observation(x_current, goal_state)
        return self.compute_actions(observation)
