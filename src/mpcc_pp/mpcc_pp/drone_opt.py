#!/usr/bin/env python
"""
ACADOS OCP solver for mpcc_pp with Track Constraint.

Stage cost (per stage k = 0 … N):

  L_k = ql · el(θk)²
      + qc(θk) · ‖ec(θk)‖²
      + Qω · ‖ωk‖²
      + rΔv · Δvθ,k²
      + RΔf · ‖Δfk‖²
      − μ · vθ,k
      + track_weight · Σ_{i=1}^{4} p(h_i(θk))   ← Track Constraint

where
  e(θk)  = pk − p^d(θk)                     position error
  el(θk) = t(θk)ᵀ · e(θk)                   lag error (scalar,  eq. 10)
  ec(θk) = e(θk) − el(θk) · t(θk)           contouring error (3D, eq. 11)

Track Constraint (mpcc_pp paper, Section IV-A, eq. 7):
  e_n = (pk − p^d)·n(θk)   — error in gate-width  direction
  e_b = (pk − p^d)·b(θk)   — error in gate-height direction

  h1 = W(θk) + e_n   ≥ 0   (inside left  wall)
  h2 = W(θk) − e_n   ≥ 0   (inside right wall)
  h3 = H(θk) + e_b   ≥ 0   (inside bottom wall)
  h4 = H(θk) − e_b   ≥ 0   (inside top   wall)

Soft barrier (paper eq. 14):  p(h) = log(1 + exp(−α·h)),  α = 100

Stage parameters (15 values, set per stage):
  [p_ref_x, p_ref_y, p_ref_z,   ← p^d(θk)        indices 0:3
   t_ref_x, t_ref_y, t_ref_z,   ← unit tangent     indices 3:6
   qc,                           ← contouring wt    index  6
   n_ref_x, n_ref_y, n_ref_z,   ← horiz normal     indices 7:10
   b_ref_x, b_ref_y, b_ref_z,   ← vert  normal     indices 10:13
   W,                            ← half-width       index  13
   H]                            ← half-height      index  14

The SQP_RTI scheme is used as in the paper (Section VII-A).
Warm-starting shifts the previous trajectory by one step (RTI practice).

Note: changing NP from 7 to 15 invalidates any previously compiled
      ACADOS C code.  ACADOS regenerates c_generated_code/ automatically
      on the first call to DroneOptimizerMpccPp().
"""

import os
import errno
import hashlib
import numpy as np
import casadi as ca
from acados_template import AcadosOcp, AcadosOcpSolver

from .parameters import get_section, load_mpcc_pp_parameters


def _makedirs(path: str):
    if not os.path.exists(path):
        try:
            os.makedirs(path)
        except OSError as e:
            if e.errno != errno.EEXIST:
                raise


class DroneOptimizerMpccPp:
    """
    ACADOS RTI solver for the mpcc_pp OCP with Track Constraint.

    Args:
        model:          DroneMpccPpModel instance.
        t_horizon:      Prediction horizon [s].
        n_nodes:        Number of shooting nodes N.
        weights:        Dict to override default cost weights (see below).
        enable_logging: Print solver info.

    Default weights
    ───────────────
    ql           — lag weight (forces accurate gate traversal)
    Qw           — body-rate regularisation (scalar, applied as Qw·I₃)
    r_dvt        — progress-acceleration cost (prevents noisy vθ)
    R_df         — rotor thrust-rate cost (scalar, applied as R_df·I₄)
    mu           — progress reward (higher → faster flight)
    track_weight — penalty weight for track constraint log-barrier terms
    """

    _DEFAULTS = dict(
        ql=7.0,
        Qw=0.0001,
        r_dvt=0.0002,
        R_df=0.0003,
        mu=7.0,
        track_weight=30.0,   # Track Constraint soft-barrier weight
    )

    # Number of parameters per stage (extended for Track Constraint)
    # [p_ref(3), t_ref(3), qc(1), n_ref(3), b_ref(3), W(1), H(1)]
    NP = 15

    # Log-barrier steepness.
    _ALPHA_TRACK = 150.0

    def __init__(self, model, t_horizon: float = 1.5, n_nodes: int = 40,
                 weights: dict = None, solver_options: dict = None,
                 track_barrier_alpha: float = None,
                 enable_logging: bool = True):
        self.T  = t_horizon
        self.N  = n_nodes
        self.nx = model.nx   # 19
        self.nu = model.nu   # 5
        self._enable_logging = enable_logging

        parameter_file = load_mpcc_pp_parameters()
        cost_parameters = get_section(parameter_file, 'cost')
        file_weights = {
            key: cost_parameters[key]
            for key in self._DEFAULTS
            if key in cost_parameters
        }
        w = {**self._DEFAULTS, **file_weights, **(weights or {})}

        solver_parameters = get_section(parameter_file, 'solver')
        solver_options = {**solver_parameters, **(solver_options or {})}
        self._alpha_track = float(
            cost_parameters.get('track_barrier_alpha', self._ALPHA_TRACK)
            if track_barrier_alpha is None
            else track_barrier_alpha)
        c = model.constraint

        os.chdir(os.path.dirname(os.path.realpath(__file__)))
        _makedirs('./acados_models')

        # ── OCP object ─────────────────────────────────────────────────
        ocp = AcadosOcp()
        ocp.model = model.model
        ocp.solver_options.N_horizon = self.N
        ocp.solver_options.tf = self.T

        # ── Per-stage parameters (15 × 1) ──────────────────────────────
        p_ref = ca.MX.sym('p_ref', 3)   # reference position       [0:3]
        t_ref = ca.MX.sym('t_ref', 3)   # unit tangent             [3:6]
        qc    = ca.MX.sym('qc',    1)   # contouring weight        [6]
        n_ref = ca.MX.sym('n_ref', 3)   # horiz tunnel normal      [7:10]
        b_ref = ca.MX.sym('b_ref', 3)   # vert  tunnel normal      [10:13]
        W     = ca.MX.sym('W',     1)   # tunnel half-width        [13]
        H     = ca.MX.sym('H',     1)   # tunnel half-height       [14]

        params = ca.vertcat(p_ref, t_ref, qc, n_ref, b_ref, W, H)   # 15 × 1

        ocp.dims.np          = self.NP
        ocp.model.p          = params
        ocp.parameter_values = np.zeros(self.NP)

        # ── State and control aliases ───────────────────────────────────
        x = model.model.x
        u = model.model.u

        p_k  = x[0:3]    # position
        w_k  = x[10:13]  # body rates
        vt_k = x[18]     # progress velocity

        dvt_k = u[0]     # progress acceleration
        df_k  = u[1:5]   # rotor thrust rates

        # ── Contouring and lag errors (Section III-D, eqs. 10–11) ───────
        e  = p_k - p_ref                   # position error
        el = ca.dot(t_ref, e)              # lag error (scalar)
        ec = e - el * t_ref                # contouring error (3D)

        # ── Track Constraint — log-barrier (mpcc_pp Section IV-A) ────────
        # Project position error onto local gate frame
        e_n = ca.dot(n_ref, e)   # error in gate-width  direction
        e_b = ca.dot(b_ref, e)   # error in gate-height direction

        # 4 halfspace margins: ≥ 0 when drone is inside the tunnel
        h1 = W + e_n   # inside left  wall  (e_n ≥ −W)
        h2 = W - e_n   # inside right wall  (e_n ≤  W)
        h3 = H + e_b   # inside bottom wall (e_b ≥ −H)
        h4 = H - e_b   # inside top   wall  (e_b ≤  H)

        # Numerically stable log-barrier: p(h) = log(1 + exp(-α·h))
        # Written as softplus(-α·h) = max(-α·h, 0) + log(1 + exp(-|α·h|))
        # This avoids exp overflow regardless of h magnitude.
        alpha = self._alpha_track

        def _barrier(h):
            z = -alpha * h
            return ca.fmax(z, 0.0) + ca.log(1.0 + ca.exp(-ca.fabs(z)))

        cost_track = w['track_weight'] * (
            _barrier(h1) + _barrier(h2) + _barrier(h3) + _barrier(h4)
        )

        # ── Stage cost ──────────────────────────────────────────────────
        cost_lag      = w['ql']    * el**2
        cost_contour  = qc         * ca.dot(ec, ec)
        cost_omega    = w['Qw']    * ca.dot(w_k, w_k)
        cost_dvt      = w['r_dvt'] * dvt_k**2
        cost_df       = w['R_df']  * ca.dot(df_k, df_k)
        cost_progress = -w['mu']   * vt_k

        stage_cost = (cost_lag + cost_contour + cost_omega
                      + cost_dvt + cost_df + cost_progress
                      + cost_track)

        # ── Terminal cost (state-only: no control increments) ────────────
        terminal_cost = (w['ql'] * el**2
                         + qc * ca.dot(ec, ec)
                         + w['Qw'] * ca.dot(w_k, w_k)
                         - w['mu'] * vt_k
                         + cost_track)

        ocp.cost.cost_type   = 'EXTERNAL'
        ocp.cost.cost_type_e = 'EXTERNAL'
        ocp.model.cost_expr_ext_cost   = stage_cost
        ocp.model.cost_expr_ext_cost_e = terminal_cost

        # ── Control constraints: u = [Δvθ, Δf1, Δf2, Δf3, Δf4] ─────────
        lbu = np.array([c.dvt_min,
                        c.df_min[0], c.df_min[1], c.df_min[2], c.df_min[3]])
        ubu = np.array([c.dvt_max,
                        c.df_max[0], c.df_max[1], c.df_max[2], c.df_max[3]])
        ocp.constraints.lbu   = lbu
        ocp.constraints.ubu   = ubu
        ocp.constraints.idxbu = np.arange(self.nu)

        # ── State constraints ────────────────────────────────────────────
        # Indices: ω @ [10,11,12], f @ [13,14,15,16], vθ @ [18]
        idx_bx = np.array([10, 11, 12, 13, 14, 15, 16, 18])
        lbx = np.concatenate([c.w_min, c.f_min, [c.vt_min]])
        ubx = np.concatenate([c.w_max, c.f_max, [c.vt_max]])
        ocp.constraints.lbx   = lbx
        ocp.constraints.ubx   = ubx
        ocp.constraints.idxbx = idx_bx

        # Terminal state constraints (same as stage)
        ocp.constraints.lbx_e   = lbx
        ocp.constraints.ubx_e   = ubx
        ocp.constraints.idxbx_e = idx_bx

        # Initial condition (overwritten each solve)
        x0 = np.zeros(self.nx)
        x0[3]    = 1.0                          # quaternion w = 1
        x0[13:17] = model.f_hover              # hover thrust
        ocp.constraints.x0 = x0

        # ── Solver options (SQP_RTI, as used in paper Section VII-A) ────
        solver_cfg = {
            'qp_solver': str(solver_options.get(
                'qp_solver', 'PARTIAL_CONDENSING_HPIPM')),
            'hessian_approx': str(solver_options.get(
                'hessian_approx', 'GAUSS_NEWTON')),
            'integrator_type': str(solver_options.get(
                'integrator_type', 'ERK')),
            'sim_method_num_stages': int(solver_options.get(
                'sim_method_num_stages', 4)),
            'sim_method_num_steps': int(solver_options.get(
                'sim_method_num_steps', 2)),
            'nlp_solver_type': str(solver_options.get(
                'nlp_solver_type', 'SQP_RTI')),
            'print_level': int(solver_options.get('print_level', 0)),
            'nlp_solver_tol_eq': float(solver_options.get(
                'nlp_solver_tol_eq', 1e-3)),
            'nlp_solver_tol_ineq': float(solver_options.get(
                'nlp_solver_tol_ineq', 1e-3)),
            'nlp_solver_tol_stat': float(solver_options.get(
                'nlp_solver_tol_stat', 1e-2)),
            'nlp_solver_tol_comp': float(solver_options.get(
                'nlp_solver_tol_comp', 1e-3)),
            'regularize_method': str(solver_options.get(
                'regularize_method', 'CONVEXIFY')),
            'levenberg_marquardt': float(solver_options.get(
                'levenberg_marquardt', 1e-2)),
            'qp_solver_warm_start': bool(solver_options.get(
                'qp_solver_warm_start', True)),
        }

        ocp.solver_options.qp_solver = solver_cfg['qp_solver']
        ocp.solver_options.hessian_approx = solver_cfg['hessian_approx']
        ocp.solver_options.integrator_type = solver_cfg['integrator_type']
        ocp.solver_options.sim_method_num_stages = solver_cfg['sim_method_num_stages']
        ocp.solver_options.sim_method_num_steps = solver_cfg['sim_method_num_steps']
        ocp.solver_options.nlp_solver_type = solver_cfg['nlp_solver_type']
        ocp.solver_options.print_level = solver_cfg['print_level']
        ocp.solver_options.nlp_solver_tol_eq = solver_cfg['nlp_solver_tol_eq']
        ocp.solver_options.nlp_solver_tol_ineq = solver_cfg['nlp_solver_tol_ineq']
        ocp.solver_options.nlp_solver_tol_stat = solver_cfg['nlp_solver_tol_stat']
        ocp.solver_options.nlp_solver_tol_comp = solver_cfg['nlp_solver_tol_comp']
        ocp.solver_options.regularize_method = solver_cfg['regularize_method']
        ocp.solver_options.levenberg_marquardt = solver_cfg['levenberg_marquardt']

        if hasattr(ocp.solver_options, 'qp_solver_warm_start'):
            ocp.solver_options.qp_solver_warm_start = int(
                solver_cfg['qp_solver_warm_start'])

        # ── Build-cache: skip recompilation when nothing changed ────────
        model_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'drone_model.py')
        with open(model_file, 'rb') as _f:
            model_hash = hashlib.md5(_f.read()).hexdigest()

        model_payload = getattr(model, 'config_hash_payload', {})
        config_str = (f"{self.T}:{self.N}:{self.nx}:{self.nu}:{self.NP}:"
                      f"{self._alpha_track}:{sorted(w.items())}:"
                      f"{sorted(solver_cfg.items())}:{model_payload}:"
                      f"{lbu.tolist()}:{ubu.tolist()}:{lbx.tolist()}:{ubx.tolist()}:"
                      f"{model_hash}")
        config_hash = hashlib.md5(config_str.encode()).hexdigest()

        hash_file = './c_generated_code/.ocp_config_hash'
        lib_file  = './c_generated_code/libacados_ocp_solver_drone_mpcc_pp.so'

        existing_hash = ''
        if os.path.exists(hash_file):
            with open(hash_file) as _f:
                existing_hash = _f.read().strip()

        needs_rebuild = (config_hash != existing_hash) or not os.path.exists(lib_file)
        # ────────────────────────────────────────────────────────────────

        json_file = './drone_mpcc_pp_acados_ocp.json'
        self.solver = AcadosOcpSolver(
            ocp, json_file=json_file,
            generate=needs_rebuild,
            build=needs_rebuild,
        )

        if needs_rebuild:
            with open(hash_file, 'w') as _f:
                _f.write(config_hash)

        # Warm-start cache
        self._x_pred: np.ndarray = None   # (N+1, nx)
        self._u_pred: np.ndarray = None   # (N,   nu)
        self._warm:   bool       = False
        self._f_hover = model.f_hover.copy()

        if enable_logging:
            if needs_rebuild:
                print(f"DroneOptimizerMpccPp: solver rebuilt (config hash: {config_hash[:8]})")
            else:
                print(f"DroneOptimizerMpccPp: reusing cached solver (config hash: {config_hash[:8]})")
            print(f"DroneOptimizerMpccPp: N={self.N}, T={self.T} s, "
                  f"dt={self.T/self.N:.3f} s")
            print(f"  nx={self.nx}, nu={self.nu}, np={self.NP}")
            print(f"  Weights: ql={w['ql']}, mu={w['mu']}, "
                  f"Qw={w['Qw']}, r_dvt={w['r_dvt']}, R_df={w['R_df']}, "
                  f"track_weight={w['track_weight']}")
            print(f"  Track barrier alpha: {self._alpha_track}")

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    def solve(self, x0: np.ndarray, stage_params: np.ndarray) -> tuple:
        """
        Execute one RTI step.

        Args:
            x0:           Current augmented state (19,).
            stage_params: Path + tunnel parameters for all N+1 stages.
                          Shape (N+1, 15).
                          Columns: [p_ref(3), t_ref(3), qc(1),
                                    n_ref(3), b_ref(3), W(1), H(1)].

        Returns:
            u_opt:  First optimal control (5,) = [Δvθ, Δf1..4].
            x_pred: Predicted state trajectory (N+1, 19).
            u_pred: Predicted control trajectory (N, 5).
        """
        # Normalise quaternion
        x0 = x0.copy()
        qn = np.linalg.norm(x0[3:7])
        if qn > 1e-8:
            x0[3:7] /= qn

        # Set per-stage parameters
        n_given = stage_params.shape[0]
        for k in range(self.N + 1):
            idx = min(k, n_given - 1)
            self.solver.set(k, 'p', stage_params[idx])

        # Pin initial condition
        self.solver.set(0, 'lbx', x0)
        self.solver.set(0, 'ubx', x0)

        # Initialise trajectory (warm or cold start)
        self._init_trajectory(x0, stage_params)

        status = self.solver.solve()
        if status not in (0, 2) and self._enable_logging:
            print(f"[mpcc_pp OCP] solver status {status}")

        # Extract solution
        x_pred = np.array([self.solver.get(k, 'x') for k in range(self.N + 1)])
        u_pred = np.array([self.solver.get(k, 'u') for k in range(self.N)])

        # Normalise quaternions in prediction
        qn = np.linalg.norm(x_pred[:, 3:7], axis=1, keepdims=True)
        qn = np.where(qn < 1e-8, 1.0, qn)
        x_pred[:, 3:7] /= qn

        self._x_pred = x_pred.copy()
        self._u_pred = u_pred.copy()
        self._warm   = bool(np.isfinite(x_pred).all() and np.isfinite(u_pred).all())

        return u_pred[0].copy(), x_pred, u_pred

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _init_trajectory(self, x0: np.ndarray, stage_params: np.ndarray):
        """Shift warm-start or set a cold-start trajectory."""
        n_given = stage_params.shape[0]

        if self._warm and self._x_pred is not None:
            # Shift by one step — standard RTI warm-start
            for k in range(self.N):
                self.solver.set(k, 'x', self._x_pred[min(k + 1, self.N)])
            self.solver.set(self.N, 'x', self._x_pred[self.N])
            for k in range(self.N - 1):
                self.solver.set(k, 'u', self._u_pred[min(k + 1, self.N - 1)])
            self.solver.set(self.N - 1, 'u', self._u_pred[self.N - 1])
        else:
            # Cold start: position from path reference, hover thrust
            for k in range(self.N + 1):
                x_k = np.zeros(self.nx)
                idx = min(k, n_given - 1)
                x_k[0:3]   = stage_params[idx, 0:3]   # position = p_ref
                x_k[3]     = 1.0                       # quaternion w = 1
                x_k[13:17] = self._f_hover             # hover thrust
                self.solver.set(k, 'x', x_k)
            u_zero = np.zeros(self.nu)
            for k in range(self.N):
                self.solver.set(k, 'u', u_zero)
