#!/usr/bin/env python
"""
MPCC controller wrapper — integrates DroneMpccModel, DroneOptimizerMpcc,
and RacingPath into the project's ControllerInterface.

Interfaces
──────────
controller_update(x, goal)          → AttitudeRate  (attitude-rate output mode,
                                        used by drone_controller_node)
trajectory_controller_update(x, T)  → AttitudeRate  (same, ignores trajectory)
mpcc_update(x)                      → ndarray (4,)  (normalised per-rotor thrust,
                                        used by the standalone race node)

State convention mapping
────────────────────────
External 13-dim (existing project):
  [p(3), v(3), q(4), ω(3)]  →  indices 0:3, 3:6, 6:10, 10:13

MPCC augmented 19-dim (paper eq. 15):
  [p(3), q(4), v(3), ω(3), f(4), θ(1), vθ(1)]
   0:3   3:7   7:10  10:13  13:17  17    18
"""

import numpy as np
from mavsdk.offboard import AttitudeRate
from interfaces import ControllerInterface

from .drone_model import DroneMpccModel
from .drone_opt import DroneOptimizerMpcc
from .path_utils import RacingPath

_MAX_RATE_DEG = 200.0   # safety clip for attitude-rate output [deg/s]


class DroneMpccController(ControllerInterface):
    """
    MPCC controller for time-optimal waypoint traversal.

    When used via drone_controller_node (controller_type='mpcc'), call
    controller_update() — it runs the full MPCC optimisation and converts
    the predicted next state into an AttitudeRate command for PX4.

    When used via the standalone race node, call mpcc_update() — it returns
    normalised per-rotor thrust (4,) for direct actuator control.

    Args:
        waypoints:      (N, 3) array of 3D gate positions (NED frame).
        gate_positions: (M, 3) gate centres for dynamic qc. Defaults to waypoints.
        t_horizon:      OCP prediction horizon [s].
        n_nodes:        Number of shooting nodes N.
        weights:        Dict to override DroneOptimizerMpcc default weights.
        sigma:          Gaussian half-width for dynamic contouring weight [m].
    """

    needs_waypoints: bool = True

    @classmethod
    def from_config(cls, config: dict) -> 'DroneMpccController':
        waypoints = config.get('waypoints')
        if waypoints is None:
            raise ValueError(
                "DroneMpccController requires 'waypoints' in the config dict. "
                "Set 'waypoints_file' in drone_controller_config.yaml."
            )
        return cls(waypoints=waypoints)

    # Expose t_H and Nnodes for compatibility with DroneController.sample_trajectory
    t_H: float
    Nnodes: int

    def __init__(self, waypoints: np.ndarray,
                 gate_positions: np.ndarray = None,
                 t_horizon: float = 1.0,
                 n_nodes: int = 20,
                 weights: dict = None,
                 sigma: float = 0.5):
        self.t_H    = t_horizon
        self.Nnodes = n_nodes
        self._dt    = t_horizon / n_nodes

        self._model = DroneMpccModel()
        self._path  = RacingPath(waypoints, gate_positions=gate_positions, sigma=sigma)
        self._opt: DroneOptimizerMpcc = None   # built lazily in initialize()

        self._weights = weights

        # Internal state: carried between control steps
        self._theta:   float      = 0.0
        self._vtheta:  float      = 0.0
        self._f_state: np.ndarray = self._model.f_hover.copy()   # rotor thrusts [N]
        self._lap:     int        = 0

        # Normalised hover thrust for fallback commands
        self._hover_thrust_norm = float(
            self._model.MASS * self._model.G / (4.0 * self._model.F_MAX))

    # ------------------------------------------------------------------
    # ControllerInterface — required abstract methods
    # ------------------------------------------------------------------

    def initialize(self):
        """Build ACADOS solver. Slow — call once before the control loop."""
        self._opt = DroneOptimizerMpcc(
            model=self._model,
            t_horizon=self.t_H,
            n_nodes=self.Nnodes,
            weights=self._weights,
            enable_logging=True,
        )
        print("[DroneMpccController] Optimizer ready. "
              f"Path length: {self._path.total_length:.2f} m")

    def controller_update(self, x_current: np.ndarray,
                          goal_position: np.ndarray) -> AttitudeRate:
        """
        Run MPCC optimisation and return an AttitudeRate command.

        The goal_position argument is ignored — MPCC follows the internal
        racing path, not a single waypoint.  Falls back to hover if the
        optimizer has not been initialised yet.
        """
        if self._opt is None:
            return self._hover_attitude_cmd()
        try:
            _, x_pred, _ = self._run_step(x_current)
            return self._x_pred_to_attitude_rate(x_pred)
        except Exception as e:
            print(f"[DroneMpccController] controller_update failed: {e}")
            return self._hover_attitude_cmd()

    def trajectory_controller_update(self, x_current: np.ndarray,
                                     goal_trajectory: np.ndarray) -> AttitudeRate:
        """
        Run MPCC optimisation and return an AttitudeRate command.

        The goal_trajectory argument is ignored — MPCC follows the internal
        racing path.  Falls back to hover if the optimizer is not ready.
        """
        if self._opt is None:
            return self._hover_attitude_cmd()
        try:
            _, x_pred, _ = self._run_step(x_current)
            return self._x_pred_to_attitude_rate(x_pred)
        except Exception as e:
            print(f"[DroneMpccController] trajectory_controller_update failed: {e}")
            return self._hover_attitude_cmd()

    # ------------------------------------------------------------------
    # Per-rotor thrust interface (race node)
    # ------------------------------------------------------------------

    def mpcc_update(self, x_ext: np.ndarray) -> np.ndarray:
        """
        Execute one MPCC control step.

        Args:
            x_ext: 13-dim drone state [p(3), v(3), q(4), ω(3)].

        Returns:
            u_norm: (4,) normalised per-rotor thrust in [0, 1].
        """
        _, x_pred, _ = self._run_step(x_ext)
        f_next = np.clip(x_pred[1, 13:17], self._model.F_MIN, self._model.F_MAX)
        return f_next / self._model.F_MAX

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def current_theta(self) -> float:
        """Current arc-length progress [m]."""
        return self._theta

    @property
    def path_length(self) -> float:
        """Total arc length of the racing path [m]."""
        return self._path.total_length

    @property
    def lap_count(self) -> int:
        """Number of completed laps."""
        return self._lap

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _run_step(self, x_ext: np.ndarray):
        """
        Core MPCC step shared by controller_update and mpcc_update.

        Updates internal theta / vtheta / f_state and returns
        (u_opt, x_pred, info) from the ACADOS solver.
        """
        x_aug = self._to_augmented(x_ext)

        # Project current position onto path (Algorithm 1)
        self._theta = self._path.find_closest_theta(x_aug[0:3], self._theta)

        # Lap detection — wrap θ to restart (closed-loop track)
        if self._path.total_length - self._theta < 0.3:
            self._theta = 0.0
            self._lap  += 1
            self._opt._warm = False   # force cold start after lap wrap

        # Compute path parameters for the OCP horizon
        stage_params = self._path.get_horizon_params(
            theta=self._theta,
            n_stages=self.Nnodes + 1,
            dt=self._dt,
            vtheta_guess=max(self._vtheta, 1.0),
        )

        # Solve MPCC OCP
        u_opt, x_pred, info = self._opt.solve(x_aug, stage_params)

        # Update internal f-state from predicted next step
        self._f_state = np.clip(
            x_pred[1, 13:17], self._model.F_MIN, self._model.F_MAX)

        # Advance θ and vθ using the first optimal control Δvθ
        delta_vt = float(u_opt[0])
        self._vtheta = float(np.clip(
            self._vtheta + delta_vt * self._dt,
            self._model.constraint.vt_min,
            self._model.constraint.vt_max,
        ))
        self._theta = min(
            self._theta + self._vtheta * self._dt,
            self._path.total_length)

        return u_opt, x_pred, info

    def _x_pred_to_attitude_rate(self, x_pred: np.ndarray) -> AttitudeRate:
        """
        Convert MPCC predicted next state to an AttitudeRate command.

        Body rates  : x_pred[1, 10:13]  [rad/s]  → deg/s for MAVSDK
        Total thrust: sum(x_pred[1, 13:17]) / (4 * F_MAX)  → normalised [0, 1]
        """
        omega_next = x_pred[1, 10:13]
        roll_deg_s  = float(np.clip(np.degrees(omega_next[0]), -_MAX_RATE_DEG, _MAX_RATE_DEG))
        pitch_deg_s = float(np.clip(np.degrees(omega_next[1]), -_MAX_RATE_DEG, _MAX_RATE_DEG))
        yaw_deg_s   = float(np.clip(np.degrees(omega_next[2]), -_MAX_RATE_DEG, _MAX_RATE_DEG))

        f_next = np.clip(x_pred[1, 13:17], self._model.F_MIN, self._model.F_MAX)
        thrust_norm = float(np.clip(
            np.sum(f_next) / (4.0 * self._model.F_MAX), 0.0, 1.0))

        return AttitudeRate(roll_deg_s, pitch_deg_s, yaw_deg_s, thrust_norm)

    def _to_augmented(self, x_ext: np.ndarray) -> np.ndarray:
        """
        Convert 13-dim external state → 19-dim MPCC augmented state.

        External: [p(3), v(3), q(4), ω(3)]
        MPCC:     [p(3), q(4), v(3), ω(3), f(4), θ(1), vθ(1)]
        """
        x_aug = np.zeros(19)
        x_aug[0:3]  = x_ext[0:3]      # p
        x_aug[3:7]  = x_ext[6:10]     # q  (w, x, y, z)
        x_aug[7:10] = x_ext[3:6]      # v
        x_aug[10:13] = x_ext[10:13]   # ω
        x_aug[13:17] = self._f_state  # f  (estimated from previous solve)
        x_aug[17]   = self._theta
        x_aug[18]   = self._vtheta
        return x_aug

    def _hover_attitude_cmd(self) -> AttitudeRate:
        """Minimal hover command for fallback situations."""
        return AttitudeRate(0.0, 0.0, 0.0, self._hover_thrust_norm)
