#!/usr/bin/env python
"""
mpcc_pp controller wrapper — integrates DroneMpccPpModel, DroneOptimizerMpccPp,
and RacingPath / GatedRacingPath into the project's ControllerInterface.

Interfaces
──────────
controller_update(x, goal)          → AttitudeRate  (attitude-rate output mode,
                                        used by drone_controller_node)
trajectory_controller_update(x, T)  → AttitudeRate  (same, ignores trajectory)
mpcc_pp_update(x)                   → ndarray (4,)  (PX4/Gazebo motor-speed commands,
                                        used by the standalone race node)

State convention mapping
────────────────────────
External 13-dim (existing project):
  [p(3), v(3), q(4), ω(3)]  →  indices 0:3, 3:6, 6:10, 10:13

mpcc_pp augmented 19-dim (paper eq. 15):
  [p(3), q(4), v(3), ω(3), f(4), θ(1), vθ(1)]
   0:3   3:7   7:10  10:13  13:17  17    18

Gate-based mpcc_pp
─────────────────
When `gates` are provided (the preferred mode for sim.launch.py with
controller_type='mpcc_pp'), a GatedRacingPath is built and the Track
Constraint is active in the OCP.  When `waypoints` are provided
(backwards-compatible fallback), a plain RacingPath is used and the
Track Constraint is effectively disabled (W=H=1000 m by default).
"""

import numpy as np
from mavsdk.offboard import AttitudeRate
from interfaces import ControllerInterface

from .drone_model import DroneMpccPpModel
from .drone_opt import DroneOptimizerMpccPp
from .parameters import (
    apply_legacy_track_overrides,
    apply_track_profile,
    get_section,
    load_mpcc_pp_parameters,
)
from .path_utils import RacingPath, GatedRacingPath


def _rgba(values, default) -> tuple[float, float, float, float]:
    """Read an RViz RGBA tuple."""
    arr = np.asarray(default if values is None else values, dtype=np.float64)
    if arr.shape != (4,):
        raise ValueError(f"RGBA values must contain 4 numbers, got shape {arr.shape}")
    return tuple(float(v) for v in arr)


class DroneMpccPpController(ControllerInterface):
    """
    mpcc_pp controller for time-optimal gate traversal.

    Preferred usage (gates YAML):
        config = {'gates': [...], 'track': {...}}
        ctrl = DroneMpccPpController.from_config(config)

    Backwards-compatible usage (waypoints array):
        ctrl = DroneMpccPpController(waypoints=np.array(...))

    Args:
        waypoints:    (N, 3) array of 3D positions, NED. Used when no gates.
        gate_positions: (M, 3) gate centres for dynamic qc (waypoints mode).
        gates:        List of gate dicts {x,y,z,yaw_deg,width,height,name}.
        track_params: Legacy per-track tuning dict from gates_config.yaml.
        track_name:   active_track name, used to apply parameters.yaml profiles.
        t_horizon:    OCP prediction horizon [s].
        n_nodes:      Number of shooting nodes N.
        weights:      Dict to override DroneOptimizerMpccPp default weights.
        sigma:        Gaussian half-width for dynamic qc weight [m] (waypoints mode).
        parameters:   Already loaded parameters.yaml content.
        parameter_file: Optional parameter YAML path. Overrides the bundled file.
    """

    needs_waypoints: bool = True

    @classmethod
    def from_config(cls, config: dict) -> 'DroneMpccPpController':
        """
        Factory method.  Detects gates vs waypoints format automatically.

        Expects config to contain either:
          {'gates': [...], 'track': {...}}   ← from gates_config.yaml
          {'waypoints': np.ndarray}          ← legacy waypoints format
        """
        if config.get('gates') is not None:
            return cls(gates=config['gates'],
                       track_params=config.get('track', {}),
                       track_name=config.get('track_name'),
                       parameter_file=config.get('parameter_file'))
        waypoints = config.get('waypoints')
        if waypoints is not None:
            return cls(waypoints=waypoints,
                       parameter_file=config.get('parameter_file'))
        raise ValueError(
            "DroneMpccPpController.from_config: config must contain "
            "'gates' (for gate-based mpcc_pp with Track Constraint) or "
            "'waypoints' (legacy mode, no tunnel constraint)."
        )

    # Expose t_H and Nnodes for compatibility with DroneController
    t_H: float
    Nnodes: int

    def __init__(self, waypoints: np.ndarray = None,
                 gate_positions: np.ndarray = None,
                 gates: list = None,
                 track_params: dict = None,
                 track_name: str = None,
                 t_horizon: float = None,
                 n_nodes: int = None,
                 weights: dict = None,
                 sigma: float = None,
                 parameters: dict = None,
                 parameter_file: str = None):
        tp = track_params or {}
        raw_parameters = (parameters if parameters is not None
                          else load_mpcc_pp_parameters(parameter_file))
        profiled_parameters = apply_track_profile(raw_parameters, track_name)
        self._parameters = apply_legacy_track_overrides(profiled_parameters, tp)

        controller_cfg = get_section(self._parameters, 'controller')
        path_cfg = get_section(self._parameters, 'path')
        contour_cfg = get_section(self._parameters, 'path', 'contouring')
        tunnel_cfg = get_section(self._parameters, 'path', 'tunnel')
        cost_cfg = get_section(self._parameters, 'cost')
        viz_cfg = get_section(self._parameters, 'visualization')

        self.t_H = float(t_horizon if t_horizon is not None
                         else controller_cfg.get('t_horizon', 0.8))
        self.Nnodes = int(n_nodes if n_nodes is not None
                          else controller_cfg.get('n_nodes', 20))
        self._dt = self.t_H / self.Nnodes

        self._min_prediction_speed = float(
            controller_cfg.get('min_prediction_speed', 1.0))
        self._lap_wrap_threshold = float(
            controller_cfg.get('lap_wrap_threshold', 0.3))
        self._debug_log_interval = max(
            1, int(controller_cfg.get('debug_log_interval', 50)))
        self._max_rate_deg = float(
            controller_cfg.get('max_attitude_rate_deg_s', 200.0))

        self.reference_path_samples = max(
            2, int(viz_cfg.get('reference_path_samples', 300)))
        self.tunnel_samples = max(
            2, int(viz_cfg.get('tunnel_samples', 160)))
        self.horizon_publish_every = max(
            1, int(viz_cfg.get('predicted_horizon_publish_every', 10)))
        self.progress_point_publish_every = max(
            1, int(viz_cfg.get('progress_point_publish_every', 1)))
        self.progress_point_scale = float(
            viz_cfg.get('progress_point_scale', 0.2))
        self.progress_point_color_rgba = _rgba(
            viz_cfg.get('progress_point_color_rgba'),
            [1.0, 1.0, 0.0, 1.0])
        self.progress_line_width = float(
            viz_cfg.get('progress_line_width', 0.03))
        self.progress_line_color_rgba = _rgba(
            viz_cfg.get('progress_line_color_rgba'),
            [1.0, 0.5, 0.0, 0.9])

        self._model = DroneMpccPpModel(
            model_params=get_section(self._parameters, 'model'))
        self._opt: DroneOptimizerMpccPp = None   # built lazily in initialize()
        weight_keys = DroneOptimizerMpccPp.WEIGHT_ORDER
        self._runtime_weights = {
            key: float(DroneOptimizerMpccPp._DEFAULTS[key])
            for key in weight_keys
        }
        self._runtime_weights.update({
            k: float(cost_cfg[k]) for k in weight_keys if k in cost_cfg
        })
        if weights:
            self._runtime_weights.update({
                k: float(v) for k, v in weights.items() if k in weight_keys
            })
        discount_value = cost_cfg.get(
            'discount_factor', DroneOptimizerMpccPp._DEFAULTS['discount_factor'])
        if weights and 'discount_factor' in weights:
            discount_value = weights['discount_factor']
        self._discount_factor = DroneOptimizerMpccPp._validate_discount_factor(
            discount_value)
        self._track_barrier_alpha = float(
            cost_cfg.get('track_barrier_alpha', DroneOptimizerMpccPp._ALPHA_TRACK))
        terminal_multiplier = cost_cfg.get(
            'terminal_cost_multiplier',
            DroneOptimizerMpccPp._DEFAULTS['terminal_cost_multiplier'])
        if weights and 'terminal_cost_multiplier' in weights:
            terminal_multiplier = weights['terminal_cost_multiplier']
        self._terminal_cost_multiplier = (
            DroneOptimizerMpccPp._validate_terminal_cost_multiplier(
                terminal_multiplier))
        self._solver_options = get_section(self._parameters, 'solver')
        self._lookup_resolution_m = float(
            path_cfg.get('lookup_resolution_m', 0.05))

        # Save gates config for deferred path rebuild with drone start position
        self._gates_config  = gates
        self._needs_start_position = gates is not None  # only for gate-based mode
        self._show_tunnel = bool(tunnel_cfg.get('show_tunnel', True))
        self._W_nominal = float(tunnel_cfg.get('W_nominal', 3.0))
        self._H_nominal = float(tunnel_cfg.get('H_nominal', 3.0))
        self._sigma_gate = float(tunnel_cfg.get('sigma_gate', 2.0))
        self._tunnel_tau = float(tunnel_cfg.get('tau', 0.95))
        self._disabled_W = float(tunnel_cfg.get('disabled_W', 1000.0))
        self._disabled_H = float(tunnel_cfg.get('disabled_H', 1000.0))
        self._qc_nom = float(contour_cfg.get('qc_nom', 0.5))
        self._qc_gate = float(contour_cfg.get('qc_gate', 10.0))
        self._sigma_qc = float(contour_cfg.get('sigma_qc', 0.5))

        if gates is not None:
            # ── Gate-based mode: Track Constraint active ──────────────────
            self._path = GatedRacingPath(
                gates,
                W_nominal  = self._W_nominal,
                H_nominal  = self._H_nominal,
                sigma_gate = self._sigma_gate,
                sigma_qc   = self._sigma_qc,
                tau_scale  = self._tunnel_tau,
                lookup_resolution_m = self._lookup_resolution_m,
            )
        elif waypoints is not None:
            # ── Waypoints fallback: Track Constraint inactive (W=H=1000) ──
            waypoint_sigma = self._sigma_qc if sigma is None else sigma
            self._path = RacingPath(
                waypoints,
                gate_positions=gate_positions,
                sigma=waypoint_sigma,
                lookup_resolution_m=self._lookup_resolution_m,
            )
        else:
            raise ValueError(
                "DroneMpccPpController requires either 'gates' or 'waypoints'.")

        # Internal state: carried between control steps
        self._theta:       float      = 0.0
        self._theta_solve: float      = 0.0
        self._vtheta:      float      = 0.0
        self._f_state: np.ndarray = self._model.f_hover.copy()
        self._lap:     int        = 0
        self._debug_counter: int  = 0
        self._solve_count: int = 0
        self._solver_fail_count: int = 0
        self._solver_status_counts: dict[int, int] = {}
        self._last_solve_info: dict | None = None
        self._last_u_opt: np.ndarray | None = None

        # Set to True after path rebuild (lead-in); node can check and republish.
        self.path_updated: bool = False

        # Debug flag — set externally (e.g. from drone_controller_node via VERBOSE_INPUT)
        self.verbose: bool = False

        # Normalized PX4 motor-speed command for fallback hover.
        self._hover_thrust_norm = float(np.mean(
            self._model.rotor_forces_to_motor_controls(self._model.f_hover)))

    # ------------------------------------------------------------------
    # ControllerInterface — required abstract methods
    # ------------------------------------------------------------------

    def initialize(self):
        """Build ACADOS solver. Slow — call once before the control loop."""
        self._opt = DroneOptimizerMpccPp(
            model=self._model,
            t_horizon=self.t_H,
            n_nodes=self.Nnodes,
            weights=self._runtime_weights,
            solver_options=self._solver_options,
            track_barrier_alpha=self._track_barrier_alpha,
            terminal_cost_multiplier=self._terminal_cost_multiplier,
            enable_logging=True,
        )
        mode = ("GatedRacingPath (Track Constraint active)"
                if isinstance(self._path, GatedRacingPath)
                else "RacingPath (Track Constraint inactive)")
        print(f"[DroneMpccPpController] Optimizer ready. "
              f"Path: {mode}, length: {self._path.total_length:.2f} m")

    def reset(self) -> None:
        """Reset path progress and warm-start state before a new run."""
        self._theta = 0.0
        self._theta_solve = 0.0
        self._vtheta = 0.0
        self._f_state = self._model.f_hover.copy()
        self._lap = 0
        self._debug_counter = 0
        self._last_x_pred = None
        self._last_u_opt = None
        self.reset_episode_stats()
        self.path_updated = False
        if self._gates_config is not None:
            self._needs_start_position = True
        if self._opt is not None:
            self._opt._warm = False

    def controller_update(self, x_current: np.ndarray,
                          goal_position: np.ndarray) -> AttitudeRate:
        """
        Run mpcc_pp optimisation and return an AttitudeRate command.

        goal_position is ignored — mpcc_pp follows the internal racing path.
        Falls back to hover if the optimizer has not been initialised yet.
        """
        if self._opt is None:
            return self._hover_attitude_cmd()
        try:
            _, x_pred, _ = self._run_step(x_current)
            return self._x_pred_to_attitude_rate(x_pred)
        except Exception as e:
            print(f"[DroneMpccPpController] controller_update failed: {e}")
            return self._hover_attitude_cmd()

    def trajectory_controller_update(self, x_current: np.ndarray,
                                     goal_trajectory: np.ndarray) -> AttitudeRate:
        """
        Run mpcc_pp optimisation and return an AttitudeRate command.

        goal_trajectory is ignored — mpcc_pp follows the internal racing path.
        """
        if self._opt is None:
            return self._hover_attitude_cmd()
        try:
            _, x_pred, _ = self._run_step(x_current)
            return self._x_pred_to_attitude_rate(x_pred)
        except Exception as e:
            print(f"[DroneMpccPpController] trajectory_controller_update failed: {e}")
            return self._hover_attitude_cmd()

    # ------------------------------------------------------------------
    # Per-rotor thrust interface (race node)
    # ------------------------------------------------------------------

    def mpcc_pp_update(self, x_ext: np.ndarray) -> np.ndarray:
        """
        Execute one mpcc_pp control step.

        Args:
            x_ext: 13-dim drone state [p(3), v(3), q(4), ω(3)].

        Returns:
            u_norm: (4,) normalized PX4/Gazebo motor-speed commands in [0, 1].
        """
        _, x_pred, _ = self._run_step(x_ext)
        f_next = np.clip(x_pred[1, 13:17], self._model.F_MIN, self._model.F_MAX)
        return self._model.rotor_forces_to_motor_controls(f_next)

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

    @property
    def last_control_input(self) -> 'np.ndarray | None':
        """Last optimal mpcc_pp input [delta_vtheta, delta_f0..3], or None."""
        u_opt = getattr(self, '_last_u_opt', None)
        if u_opt is None:
            return None
        return u_opt.copy()

    def sync_augmented_state(self, x_aug: np.ndarray) -> None:
        """Sync internal thrust estimate from a 19-dim simulated MPCC++ state."""
        x_aug = np.asarray(x_aug, dtype=np.float64).reshape(-1)
        if x_aug.shape != (19,) or not np.isfinite(x_aug).all():
            return
        self._f_state = np.clip(
            x_aug[13:17], self._model.F_MIN, self._model.F_MAX)

    def sample_reference_path(self, n: int = None) -> np.ndarray:
        """Return (n, 3) NED positions sampled uniformly along the reference spline."""
        if n is None:
            n = self.reference_path_samples
        thetas = np.linspace(0.0, self._path.total_length, n)
        return np.array([self._path.position(t) for t in thetas])

    def sample_tunnel(self, n: int = None) -> 'dict[str, np.ndarray] | None':
        """
        Return sampled tunnel geometry for RViz visualisation.

        Returns None for the legacy RacingPath mode, where the OCP receives
        W=H=1000 m as a practical way to disable the track constraint.
        """
        if not self._show_tunnel:
            return None
        if not isinstance(self._path, GatedRacingPath):
            return None

        if n is None:
            n = self.tunnel_samples
        thetas = np.linspace(0.0, self._path.total_length, n)
        positions = np.zeros((n, 3), dtype=np.float64)
        normals = np.zeros((n, 3), dtype=np.float64)
        binormals = np.zeros((n, 3), dtype=np.float64)
        widths = np.zeros(n, dtype=np.float64)
        heights = np.zeros(n, dtype=np.float64)

        def ring_corners(p, n_axis, b_axis, W, H):
            return np.array([
                p - W * n_axis - H * b_axis,
                p + W * n_axis - H * b_axis,
                p + W * n_axis + H * b_axis,
                p - W * n_axis + H * b_axis,
            ], dtype=np.float64)

        prev_ring = None
        for i, th in enumerate(thetas):
            p = self._path.position(th)
            n_axis, b_axis = self._path.normal_binormal(th)
            W = self._path.tunnel_width(th)
            H = self._path.tunnel_height(th)

            # The tunnel constraints are sign-symmetric in n/b, but the RViz
            # surface connects ring corners by index.  Pick the equivalent
            # frame orientation whose corners move least from the previous ring
            # so vertically stacked gates do not loft top-to-top awkwardly.
            if prev_ring is not None:
                candidates = []
                for n_sign in (1.0, -1.0):
                    for b_sign in (1.0, -1.0):
                        ring = ring_corners(
                            p, n_sign * n_axis, b_sign * b_axis, W, H)
                        score = float(np.sum((ring - prev_ring) ** 2))
                        candidates.append((score, n_sign, b_sign, ring))
                _, n_sign, b_sign, ring = min(candidates, key=lambda item: item[0])
                n_axis = n_sign * n_axis
                b_axis = b_sign * b_axis
            else:
                ring = ring_corners(p, n_axis, b_axis, W, H)

            positions[i] = p
            normals[i] = n_axis
            binormals[i] = b_axis
            widths[i] = W
            heights[i] = H
            prev_ring = ring

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

    def set_tuning_parameters(self, params: dict) -> None:
        """
        Update BO-tuned parameters without rebuilding ACADOS.

        The OCP receives these weights as runtime stage parameters.  The
        contouring values remain path parameters because qc is computed along
        the horizon from qc_nom/qc_gate and the gate-centered Gaussian profile.
        """
        if not params:
            return

        allowed = set(DroneOptimizerMpccPp.WEIGHT_ORDER)
        allowed.update({'discount_factor', 'qc_nom', 'qc_gate', 'qc_gate_ratio'})
        unknown = sorted(set(params) - allowed)
        if unknown:
            raise ValueError(f"Unknown MPCC++ tuning parameter(s): {unknown}")

        for key in DroneOptimizerMpccPp.WEIGHT_ORDER:
            if key in params:
                self._runtime_weights[key] = self._positive_float(
                    key, params[key])
        if 'discount_factor' in params:
            self._discount_factor = DroneOptimizerMpccPp._validate_discount_factor(
                params['discount_factor'])

        qc_nom = self._positive_float(
            'qc_nom', params.get('qc_nom', self._qc_nom))
        if 'qc_gate' in params:
            qc_gate = self._positive_float('qc_gate', params['qc_gate'])
        elif 'qc_gate_ratio' in params:
            ratio = self._positive_float(
                'qc_gate_ratio', params['qc_gate_ratio'])
            qc_gate = qc_nom * ratio
        else:
            qc_gate = self._positive_float('qc_gate', self._qc_gate)

        self._qc_nom = qc_nom
        self._qc_gate = qc_gate

    def get_tuning_parameters(self) -> dict:
        """Return the currently active runtime tuning parameters."""
        params = dict(self._runtime_weights)
        params['discount_factor'] = float(self._discount_factor)
        params['qc_nom'] = float(self._qc_nom)
        params['qc_gate'] = float(self._qc_gate)
        params['qc_gate_ratio'] = float(self._qc_gate / self._qc_nom)
        return params

    def reset_episode_stats(self) -> None:
        """Reset solver counters used by the Optuna episode evaluator."""
        self._solve_count = 0
        self._solver_fail_count = 0
        self._solver_status_counts = {}
        self._last_solve_info = None

    def get_solver_stats(self) -> dict:
        """Return cumulative solver diagnostics for the current episode."""
        fail_rate = (self._solver_fail_count / self._solve_count
                     if self._solve_count else 0.0)
        last_status = None
        if self._last_solve_info is not None:
            last_status = self._last_solve_info.get('status')
        return {
            'solve_count': self._solve_count,
            'solver_fail_count': self._solver_fail_count,
            'solver_fail_rate': float(fail_rate),
            'status_counts': dict(self._solver_status_counts),
            'last_status': last_status,
        }

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _run_step(self, x_ext: np.ndarray):
        """
        Core mpcc_pp step shared by controller_update and mpcc_pp_update.

        Uses get_tunnel_params() for the 16-column path/tunnel geometry, then
        appends runtime tuning weights and per-stage cost discounts expected
        by the OCP.  The final geometry column is theta_ref; ACADOS uses it
        to update p_ref symbolically from the optimized theta state.  For plain
        RacingPath (no gates),
        get_tunnel_params() returns large W=H=1000 so the barrier is inactive.
        """
        # On the first call in gate-based mode: rebuild path with drone start
        # position so the reference trajectory begins at the drone, not gate 1.
        if self._needs_start_position:
            drone_pos = np.asarray(x_ext[0:3], dtype=np.float64)
            self._path = GatedRacingPath(
                self._gates_config,
                W_nominal     = self._W_nominal,
                H_nominal     = self._H_nominal,
                sigma_gate    = self._sigma_gate,
                sigma_qc      = self._sigma_qc,
                tau_scale     = self._tunnel_tau,
                start_position = drone_pos,
                lookup_resolution_m = self._lookup_resolution_m,
            )
            self._theta   = 0.0
            self._vtheta  = 0.0
            self._needs_start_position = False
            self.path_updated = True   # signal to node to republish the path
            print(f"[DroneMpccPpController] Lead-in: start at "
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
        if self._path.total_length - self._theta < self._lap_wrap_threshold:
            circuit_start = (self._path.circuit_start_theta
                             if isinstance(self._path, GatedRacingPath)
                             else 0.0)
            self._theta = circuit_start
            self._lap  += 1
            self._opt._warm = False   # force cold start after lap wrap

        # Compute path + tunnel geometry and append runtime cost parameters.
        # import time
        # tic = time.time()
        stage_params = self._path.get_tunnel_params(
            theta        = self._theta,
            n_stages     = self.Nnodes + 1,
            dt           = self._dt,
            vtheta_guess = max(self._vtheta, self._min_prediction_speed),
            qc_nom       = self._qc_nom,
            qc_gate      = self._qc_gate,
            W_default    = self._disabled_W,
            H_default    = self._disabled_H,
        )
        # tac = time.time()
        # print(f"Took {tac - tic:.5f} s")
        stage_params = self._append_cost_params(stage_params)

        # Solve mpcc_pp OCP
        u_opt, x_pred, info = self._opt.solve(x_aug, stage_params)
        self._record_solver_info(info)
        self._last_u_opt = np.asarray(u_opt, dtype=np.float64).copy()
        self._last_x_pred = x_pred   # expose for debug / visualisation

        self._debug_counter += 1
        if self.verbose and self._debug_counter % self._debug_log_interval == 0:
            f_raw = x_pred[1, 13:17]
            mean_force = np.array([np.mean(f_raw)], dtype=np.float64)
            thrust_raw = float(
                self._model.rotor_forces_to_motor_controls(mean_force)[0])
            # print(f"[DEBUG] mpcc_pp solver: theta={self._theta:.3f} m, "
            #       f"vtheta={self._vtheta:.3f} m/s, lap={self._lap}, "
            #       f"f_rotor={np.round(f_raw, 3).tolist()}, "
            #       f"motor_norm={thrust_raw:.3f}")

        # print the linear velocity norm of every step
        # v_pred = x_pred[:, 7:10]
        # v_norm = np.linalg.norm(v_pred, axis=1)
        # print(f"[DEBUG] mpcc_pp predicted velocity norms: {np.round(v_norm, 3).tolist()}")
        # print the progress velocity at every step
        # vtheta_pred = x_pred[:, 18]
        # print(f"[DEBUG] mpcc_pp predicted progress velocities: {np.round(vtheta_pred, 3).tolist()}")
        
        # Update internal f-state from predicted next step
        # print(f"[DEBUG] Updating internal f_state from x_pred: {x_pred[1, 13:17]}")
        self._f_state = np.clip(
            x_pred[1, 13:17], self._model.F_MIN, self._model.F_MAX)
        # print(f"[DEBUG] Updated internal f_state: {self._f_state}\n\n")

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

        # print(f"\n\n x_pred[0]: {x_pred[0]}\n u_opt: {u_opt}\n x_pred[1]: {x_pred[1]}\n\n")

        return u_opt, x_pred, info

    @staticmethod
    def _positive_float(name: str, value) -> float:
        """Convert a tuning value to a finite positive float."""
        converted = float(value)
        if not np.isfinite(converted) or converted <= 0.0:
            raise ValueError(f"{name} must be a finite positive value")
        return converted

    def _append_cost_params(self, stage_params: np.ndarray) -> np.ndarray:
        """Append runtime cost weights and discounts to stage parameters."""
        params = np.asarray(stage_params, dtype=np.float64)
        weights = np.array([
            self._runtime_weights[key]
            for key in DroneOptimizerMpccPp.WEIGHT_ORDER
        ], dtype=np.float64)
        repeated = np.repeat(weights.reshape(1, -1), params.shape[0], axis=0)
        discounts = DroneOptimizerMpccPp._discount_vector(
            params.shape[0], self._discount_factor)
        return np.hstack([params, repeated, discounts])

    def _record_solver_info(self, info: dict) -> None:
        """Accumulate solver status counters for tuning metrics."""
        self._solve_count += 1
        status = int(info.get('status', -1))
        self._solver_status_counts[status] = (
            self._solver_status_counts.get(status, 0) + 1)
        if not bool(info.get('success', False)):
            self._solver_fail_count += 1
        self._last_solve_info = {
            key: value for key, value in info.items() if key != 'u_pred'
        }

    def _x_pred_to_attitude_rate(self, x_pred: np.ndarray) -> AttitudeRate:
        """
        Convert mpcc_pp predicted next state to an AttitudeRate command.

        Body rates  : x_pred[1, 10:13]  [rad/s]  → deg/s for MAVSDK
        Total thrust: mean rotor force → Gazebo motor-speed command.
        """
        omega_next = x_pred[1, 10:13]
        roll_deg_s = float(np.clip(
            np.degrees(omega_next[0]), -self._max_rate_deg, self._max_rate_deg))
        pitch_deg_s = float(np.clip(
            np.degrees(omega_next[1]), -self._max_rate_deg, self._max_rate_deg))
        yaw_deg_s = float(np.clip(
            np.degrees(omega_next[2]), -self._max_rate_deg, self._max_rate_deg))

        f_next = np.clip(x_pred[1, 13:17], self._model.F_MIN, self._model.F_MAX)
        mean_force = np.array([np.mean(f_next)], dtype=np.float64)
        thrust_norm = float(
            self._model.rotor_forces_to_motor_controls(mean_force)[0])

        if self.verbose and self._debug_counter % self._debug_log_interval == 0:
            print(f"[DEBUG] mpcc_pp AttitudeRate: thrust={thrust_norm:.3f}, "
                  f"wx={roll_deg_s:.1f} deg/s, wy={pitch_deg_s:.1f} deg/s, "
                  f"wz={yaw_deg_s:.1f} deg/s")

        return AttitudeRate(roll_deg_s, pitch_deg_s, yaw_deg_s, thrust_norm)

    def _to_augmented(self, x_ext: np.ndarray) -> np.ndarray:
        """
        Convert 13-dim external state → 19-dim mpcc_pp augmented state.

        External: [p(3), v(3), q(4), ω(3)]
        mpcc_pp:   [p(3), q(4), v(3), ω(3), f(4), θ(1), vθ(1)]
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
