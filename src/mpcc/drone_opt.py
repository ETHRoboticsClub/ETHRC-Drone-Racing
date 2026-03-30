#!/usr/bin/env python
"""
ACADOS OCP solver for MPCC — eq. (17) of Romero et al., T-RO 2022.

Cost function (per stage k = 0 … N):

  L_k = ql · el(θk)²
      + qc(θk) · ‖ec(θk)‖²
      + Qω · ‖ωk‖²
      + rΔv · Δvθ,k²
      + RΔf · ‖Δfk‖²
      − μ · vθ,k

where
  e(θk)  = pk − p^d(θk)                     position error
  el(θk) = t(θk)ᵀ · e(θk)                   lag error (scalar,  eq. 10)
  ec(θk) = e(θk) − el(θk) · t(θk)           contouring error (3D, eq. 11)

Stage parameters (7 values, set per stage):
  [p_ref_x, p_ref_y, p_ref_z,   ← p^d(θk)
   t_ref_x, t_ref_y, t_ref_z,   ← unit tangent t(θk)
   qc]                           ← dynamic contouring weight

The SQP_RTI scheme is used as in the paper (Section VII-A).
Warm-starting shifts the previous trajectory by one step (RTI practice).
"""

import os
import errno
import numpy as np
import casadi as ca
from acados_template import AcadosOcp, AcadosOcpSolver


def _makedirs(path: str):
    if not os.path.exists(path):
        try:
            os.makedirs(path)
        except OSError as e:
            if e.errno != errno.EEXIST:
                raise


class DroneOptimizerMpcc:
    """
    ACADOS RTI solver for the MPCC OCP.

    Args:
        model:          DroneMpccModel instance.
        t_horizon:      Prediction horizon [s].
        n_nodes:        Number of shooting nodes N.
        weights:        Dict to override default cost weights (see below).
        enable_logging: Print solver info.

    Default weights
    ───────────────
    ql      — lag weight (high value forces accurate gate traversal)
    Qw      — body-rate regularisation (scalar, applied as Qw·I₃)
    r_dvt   — progress-acceleration cost (prevents noisy vθ)
    R_df    — rotor thrust-rate cost (scalar, applied as R_df·I₄)
    mu      — progress reward (higher → faster flight)
    """

    _DEFAULTS = dict(
        ql=10.0,
        Qw=0.1,
        r_dvt=0.1,
        R_df=0.01,
        mu=10.0,
    )

    # Number of parameters per stage: [p_ref(3), t_ref(3), qc(1)]
    NP = 7

    def __init__(self, model, t_horizon: float = 1.0, n_nodes: int = 20,
                 weights: dict = None, enable_logging: bool = True):
        self.T  = t_horizon
        self.N  = n_nodes
        self.nx = model.nx   # 19
        self.nu = model.nu   # 5
        self._enable_logging = enable_logging

        w = {**self._DEFAULTS, **(weights or {})}
        c = model.constraint

        os.chdir(os.path.dirname(os.path.realpath(__file__)))
        _makedirs('./acados_models')

        # ── OCP object ─────────────────────────────────────────────────
        ocp = AcadosOcp()
        ocp.model = model.model
        ocp.solver_options.N_horizon = self.N
        ocp.solver_options.tf = self.T

        # ── Per-stage parameters: [p_ref(3), t_ref(3), qc(1)] ──────────
        p_ref = ca.MX.sym('p_ref', 3)
        t_ref = ca.MX.sym('t_ref', 3)
        qc    = ca.MX.sym('qc',    1)
        params = ca.vertcat(p_ref, t_ref, qc)    # 7 × 1

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

        # ── Stage cost ──────────────────────────────────────────────────
        cost_lag      = w['ql']    * el**2
        cost_contour  = qc         * ca.dot(ec, ec)
        cost_omega    = w['Qw']    * ca.dot(w_k, w_k)
        cost_dvt      = w['r_dvt'] * dvt_k**2
        cost_df       = w['R_df']  * ca.dot(df_k, df_k)
        cost_progress = -w['mu']   * vt_k

        stage_cost = (cost_lag + cost_contour + cost_omega
                      + cost_dvt + cost_df + cost_progress)

        # ── Terminal cost (state-only: no control increments) ────────────
        terminal_cost = (w['ql'] * el**2
                         + qc * ca.dot(ec, ec)
                         + w['Qw'] * ca.dot(w_k, w_k)
                         - w['mu'] * vt_k)

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
        ocp.solver_options.qp_solver             = 'PARTIAL_CONDENSING_HPIPM'
        ocp.solver_options.hessian_approx        = 'GAUSS_NEWTON'
        ocp.solver_options.integrator_type       = 'ERK'
        ocp.solver_options.sim_method_num_stages = 4   # RK4
        ocp.solver_options.sim_method_num_steps  = 2
        ocp.solver_options.nlp_solver_type       = 'SQP_RTI'
        ocp.solver_options.print_level           = 0
        ocp.solver_options.nlp_solver_tol_eq     = 1e-3
        ocp.solver_options.nlp_solver_tol_ineq   = 1e-3
        ocp.solver_options.nlp_solver_tol_stat   = 1e-2
        ocp.solver_options.nlp_solver_tol_comp   = 1e-3
        ocp.solver_options.regularize_method     = 'CONVEXIFY'
        ocp.solver_options.levenberg_marquardt   = 1e-2

        if hasattr(ocp.solver_options, 'qp_solver_warm_start'):
            ocp.solver_options.qp_solver_warm_start = 1

        json_file = './drone_mpcc_acados_ocp.json'
        self.solver = AcadosOcpSolver(ocp, json_file=json_file)

        # Warm-start cache
        self._x_pred: np.ndarray = None   # (N+1, nx)
        self._u_pred: np.ndarray = None   # (N,   nu)
        self._warm:   bool       = False
        self._f_hover = model.f_hover.copy()

        if enable_logging:
            print(f"DroneOptimizerMpcc: N={self.N}, T={self.T} s, "
                  f"dt={self.T/self.N:.3f} s")
            print(f"  nx={self.nx}, nu={self.nu}, np={self.NP}")
            print(f"  Weights: ql={w['ql']}, mu={w['mu']}, "
                  f"Qw={w['Qw']}, r_dvt={w['r_dvt']}, R_df={w['R_df']}")

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    def solve(self, x0: np.ndarray, stage_params: np.ndarray) -> tuple:
        """
        Execute one RTI step.

        Args:
            x0:           Current augmented state (19,).
            stage_params: Path parameters for all N+1 stages, shape (N+1, 7).
                          Columns: [p_ref(3), t_ref(3), qc(1)].

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
            print(f"[MPCC OCP] solver status {status}")

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
