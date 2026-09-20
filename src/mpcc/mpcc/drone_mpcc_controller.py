#!/usr/bin/env python
"""
MPCC controller wrapper — integrates DroneMpccModel, DroneOptimizerMpcc,
and RacingPath / GatedRacingPath into the project's ControllerInterface.

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

Gate-based MPCC
───────────────
When `gates` are provided (the preferred mode for sim.launch.py with
controller_type='mpcc'), a GatedRacingPath is built and the Track
Constraint is active in the OCP.  When `waypoints` are provided
(backwards-compatible fallback), a plain RacingPath is used and the
Track Constraint is effectively disabled (W=H=1000 m by default).
"""

import time

import numpy as np
from mavsdk.offboard import AttitudeRate
from interfaces import ControllerInterface

from .drone_model import DroneMpccModel
from .drone_opt import DroneOptimizerMpcc
from .path_utils import RacingPath, GatedRacingPath

try:
    from mpcc_pp.model_logger import ModelValidationLogger as _ModelValidationLogger
    _LOGGER_CLS = _ModelValidationLogger
except ImportError:
    _LOGGER_CLS = None

_MAX_RATE_DEG = 200.0   # safety clip for attitude-rate output [deg/s]


class DroneMpccController(ControllerInterface):
    """
    MPCC controller for time-optimal gate traversal.

    Preferred usage (gates YAML):
        config = {'gates': [...], 'track': {...}}
        ctrl = DroneMpccController.from_config(config)

    Backwards-compatible usage (waypoints array):
        ctrl = DroneMpccController(waypoints=np.array(...))

    Args:
        waypoints:    (N, 3) array of 3D positions, NED. Used when no gates.
        gate_positions: (M, 3) gate centres for dynamic qc (waypoints mode).
        gates:        List of gate dicts {x,y,z,yaw_deg,width,height,name}.
        track_params: Dict with tunnel params (W_nominal, H_nominal, …).
        t_horizon:    OCP prediction horizon [s].
        n_nodes:      Number of shooting nodes N.
        weights:      Dict to override DroneOptimizerMpcc default weights.
        sigma:        Gaussian half-width for dynamic qc weight [m] (waypoints mode).
    """

    needs_waypoints: bool = True

    @classmethod
    def from_config(cls, config: dict) -> 'DroneMpccController':
        """
        Factory method.  Detects gates vs waypoints format automatically.

        Expects config to contain either:
          {'gates': [...], 'track': {...}}   ← from gates_config.yaml
          {'waypoints': np.ndarray}          ← legacy waypoints format
        """
        if config.get('gates') is not None:
            return cls(gates=config['gates'],
                       track_params=config.get('track', {}))
        waypoints = config.get('waypoints')
        if waypoints is not None:
            return cls(waypoints=waypoints)
        raise ValueError(
            "DroneMpccController.from_config: config must contain "
            "'gates' (for gate-based MPCC with Track Constraint) or "
            "'waypoints' (legacy mode, no tunnel constraint)."
        )

    # Expose t_H and Nnodes for compatibility with DroneController
    t_H: float
    Nnodes: int

    def __init__(self, waypoints: np.ndarray = None,
                 gate_positions: np.ndarray = None,
                 gates: list = None,
                 track_params: dict = None,
                 t_horizon: float = 0.8,
                 n_nodes: int = 20,
                 weights: dict = None,
                 sigma: float = 0.5):
        self.t_H    = t_horizon
        self.Nnodes = n_nodes
        self._dt    = t_horizon / n_nodes

        self._model = DroneMpccModel()
        self._opt: DroneOptimizerMpcc = None   # built lazily in initialize()
        self._weights = weights

        tp = track_params or {}

        # Save gates config for deferred path rebuild with drone start position
        self._gates_config  = gates
        self._track_params  = tp
        self._needs_start_position = gates is not None  # only for gate-based mode
        self._show_tunnel = bool(tp.get('SHOW_TUNNEL', True))

        if gates is not None:
            # ── Gate-based mode: Track Constraint active ──────────────────
            self._path = GatedRacingPath(
                gates,
                W_nominal  = tp.get('W_nominal',  3.0),
                H_nominal  = tp.get('H_nominal',  3.0),
                sigma_gate = tp.get('sigma_gate', 2.0),
                sigma_qc   = tp.get('sigma_qc',   0.5),
            )
            # Merge track_weight from YAML into weights dict for the OCP
            if 'track_weight' in tp and self._weights is None:
                self._weights = {'track_weight': tp['track_weight']}
            elif 'track_weight' in tp:
                self._weights.setdefault('track_weight', tp['track_weight'])
        elif waypoints is not None:
            # ── Waypoints fallback: Track Constraint inactive (W=H=1000) ──
            self._path = RacingPath(waypoints,
                                    gate_positions=gate_positions,
                                    sigma=sigma)
        else:
            raise ValueError(
                "DroneMpccController requires either 'gates' or 'waypoints'.")

        # Store qc params from track config for the control loop
        self._qc_nom  = tp.get('qc_nom',  0.5)
        self._qc_gate = tp.get('qc_gate', 10.0)

        # Internal state: carried between control steps
        self._theta:       float      = 0.0
        self._theta_solve: float      = 0.0
        self._vtheta:      float      = 0.0
        self._f_state: np.ndarray = self._model.f_hover.copy()
        self._lap:     int        = 0
        self._debug_counter: int  = 0
        self.horizon_publish_every = 10
        self.progress_point_publish_every = 10

        # Set to True after path rebuild (lead-in); node can check and republish.
        self.path_updated: bool = False

        # Debug flag — set externally (e.g. from drone_controller_node via VERBOSE_INPUT)
        self.verbose: bool = False

        # Model-validation logger — activated by drone_controller_node when
        # the model_logging parameter is True.
        self._logger = None

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
        mode = ("GatedRacingPath (Track Constraint active)"
                if isinstance(self._path, GatedRacingPath)
                else "RacingPath (Track Constraint inactive)")
        print(f"[DroneMpccController] Optimizer ready. "
              f"Path: {mode}, length: {self._path.total_length:.2f} m")

    def enable_model_logging(self, data_dir: str | None = None) -> None:
        """Activate model-validation CSV logging.  Safe to call multiple times."""
        if _LOGGER_CLS is None:
            print('[DroneMpccController] model_logging requested but mpcc_pp package '
                  'not available — logging disabled.')
            return
        if self._logger is not None:
            self._logger.close()
        self._logger = _LOGGER_CLS(data_dir=data_dir)

    def disable_model_logging(self) -> None:
        """Deactivate and close the model-validation logger."""
        if self._logger is not None:
            self._logger.close()
            self._logger = None

    def reset(self) -> None:
        """Reset path progress and warm-start state before a new run."""
        self._theta = 0.0
        self._theta_solve = 0.0
        self._vtheta = 0.0
        self._f_state = self._model.f_hover.copy()
        self._lap = 0
        self._debug_counter = 0
        self._last_x_pred = None
        self.path_updated = False
        if self._gates_config is not None:
            self._needs_start_position = True
        if self._opt is not None:
            self._opt._warm = False
        if self._logger is not None:
            self._logger.reset()

    def controller_update(self, x_current: np.ndarray,
                          goal_position: np.ndarray) -> AttitudeRate:
        """
        Run MPCC optimisation and return an AttitudeRate command.

        goal_position is ignored — MPCC follows the internal racing path.
        Falls back to hover if the optimizer has not been initialised yet.
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

        goal_trajectory is ignored — MPCC follows the internal racing path.
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
    def last_predicted_positions(self) -> 'np.ndarray | None':
        """(N+1, 3) predicted positions in NED from the last OCP solve, or None."""
        pred = getattr(self, '_last_x_pred', None)
        if pred is None:
            return None
        return pred[:, 0:3]

    def sample_reference_path(self, n: int = 300) -> np.ndarray:
        """Return (n, 3) NED positions sampled uniformly along the reference spline."""
        thetas = np.linspace(0.0, self._path.total_length, n)
        return np.array([self._path.position(t) for t in thetas])

    def sample_tunnel(self, n: int = 160) -> 'dict[str, np.ndarray] | None':
        """
        Return sampled tunnel geometry for RViz visualisation.

        Returns None for the legacy RacingPath mode, where the OCP receives
        W=H=1000 m as a practical way to disable the track constraint.
        """
        if not self._show_tunnel:
            return None
        if not isinstance(self._path, GatedRacingPath):
            return None

        thetas = np.linspace(0.0, self._path.total_length, n)
        positions = np.zeros((n, 3), dtype=np.float64)
        normals = np.zeros((n, 3), dtype=np.float64)
        binormals = np.zeros((n, 3), dtype=np.float64)
        widths = np.zeros(n, dtype=np.float64)
        heights = np.zeros(n, dtype=np.float64)

        for i, th in enumerate(thetas):
            positions[i] = self._path.position(th)
            normals[i], binormals[i] = self._path.normal_binormal(th)
            widths[i] = self._path.tunnel_width(th)
            heights[i] = self._path.tunnel_height(th)

        return {
            'positions': positions,
            'normals': normals,
            'binormals': binormals,
            'widths': widths,
            'heights': heights,
        }

    @property
    def current_theta(self) -> float:
        """Current arc-length progress [m]."""
        return self._theta

    @property
    def current_path_position(self) -> np.ndarray:
        """NED position on the reference path projected from the drone."""
        return self._path.position(self._theta_solve)

    @property
    def path_length(self) -> float:
        """Total arc length of the racing path [m]."""
        return self._path.total_length

    @property
    def lap_count(self) -> int:
        """Number of completed laps."""
        return self._lap

    @property
    def gate_thetas(self) -> np.ndarray | None:
        """Arc-length positions of gates on the current path, if available."""
        if isinstance(self._path, GatedRacingPath):
            return np.asarray(self._path.gate_thetas, dtype=np.float64)
        return None

    @property
    def gate_positions(self) -> np.ndarray | None:
        """Gate centre positions in NED, if gate geometry is available."""
        if self._gates_config is None:
            return None
        return np.asarray(
            [[g['x'], g['y'], g['z']] for g in self._gates_config],
            dtype=np.float64,
        )

    @property
    def gate_miss_distances(self) -> np.ndarray | None:
        """Conservative per-gate tolerance from the aperture dimensions."""
        if self._gates_config is None:
            return None
        return np.asarray([
            min(float(g.get('width', 2.0)),
                float(g.get('height', 2.0))) / 2.0
            for g in self._gates_config
        ], dtype=np.float64)

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _run_step(self, x_ext: np.ndarray):
        """
        Core MPCC step shared by controller_update and mpcc_update.

        Uses get_tunnel_params() which provides the full 15-column parameter
        matrix for the Track-Constraint OCP.  For plain RacingPath (no gates),
        get_tunnel_params() returns large W=H=1000 so the barrier is inactive.
        """
        step_time = time.monotonic()

        # On the first call in gate-based mode: rebuild path with drone start
        # position so the reference trajectory begins at the drone, not gate 1.
        if self._needs_start_position:
            drone_pos = np.asarray(x_ext[0:3], dtype=np.float64)
            tp = self._track_params
            self._path = GatedRacingPath(
                self._gates_config,
                W_nominal    = tp.get('W_nominal',  3.0),
                H_nominal    = tp.get('H_nominal',  3.0),
                sigma_gate   = tp.get('sigma_gate', 2.0),
                sigma_qc     = tp.get('sigma_qc',   0.5),
                start_position = drone_pos,
            )
            self._theta   = 0.0
            self._vtheta  = 0.0
            self._needs_start_position = False
            self.path_updated = True   # signal to node to republish the path
            print(f"[DroneMpccController] Lead-in: start at "
                  f"({drone_pos[0]:.2f}, {drone_pos[1]:.2f}, {drone_pos[2]:.2f}), "
                  f"circuit starts at θ={self._path.circuit_start_theta:.2f} m")

        x_aug = self._to_augmented(x_ext)

        # Project current position onto path (Algorithm 1)
        self._theta = self._path.find_closest_theta(x_aug[0:3], self._theta)
        self._theta_solve = self._theta
        # Pin OCP initial θ to the projected value (not the stale vθ-advanced value
        # that _to_augmented set). This keeps x_aug and stage_params consistent.
        x_aug[17] = self._theta

        # Lap detection — wrap θ to gate 1 (skip lead-in on subsequent laps)
        if self._path.total_length - self._theta < 0.3:
            circuit_start = (self._path.circuit_start_theta
                             if isinstance(self._path, GatedRacingPath)
                             else 0.0)
            self._theta = circuit_start
            self._lap  += 1
            self._opt._warm = False   # force cold start after lap wrap

        # Compute path + tunnel parameters for the OCP horizon (15 columns)
        stage_params = self._path.get_tunnel_params(
            theta        = self._theta,
            n_stages     = self.Nnodes + 1,
            dt           = self._dt,
            vtheta_guess = max(self._vtheta, 1.0),
            qc_nom       = self._qc_nom,
            qc_gate      = self._qc_gate,
        )

        # Solve MPCC OCP
        u_opt, x_pred, info = self._opt.solve(x_aug, stage_params)
        self._last_x_pred = x_pred   # expose for debug / visualisation

        if self._logger is not None:
            self._logger.record(step_time, x_aug, u_opt, x_pred[1], x_ext)

        self._debug_counter += 1
        if self.verbose and self._debug_counter % 50 == 0:
            f_raw = x_pred[1, 13:17]
            thrust_raw = float(np.sum(f_raw) / (4.0 * self._model.F_MAX))
            print(f"[DEBUG] MPCC solver: theta={self._theta:.3f} m, "
                  f"vtheta={self._vtheta:.3f} m/s, lap={self._lap}, "
                  f"f_rotor={np.round(f_raw, 3).tolist()}, "
                  f"thrust_norm={thrust_raw:.3f}")

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

        if self.verbose and self._debug_counter % 50 == 0:
            print(f"[DEBUG] MPCC AttitudeRate: thrust={thrust_norm:.3f}, "
                  f"wx={roll_deg_s:.1f} deg/s, wy={pitch_deg_s:.1f} deg/s, "
                  f"wz={yaw_deg_s:.1f} deg/s")

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
