#!/usr/bin/env python
"""
ACADOS OCP solver wrapper for drone MPC.
Control inputs: per-rotor normalized thrust u[0..3] in [0, 1].
Output of compute_control_once: [thrust_N, wx, wy, wz] (attitude_rate compatible).

The optimizer internally works with per-rotor inputs and full Euler dynamics,
then converts the optimal solution via the mixing matrix before returning.
"""
import os
import sys
import shutil
import errno
import numpy as np
import casadi as ca
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver


def safe_mkdir_recursive(directory, overwrite=False):
    """Create directory recursively if it does not exist."""
    if not os.path.exists(directory):
        try:
            os.makedirs(directory)
        except OSError as exc:
            if exc.errno == errno.EEXIST and os.path.isdir(directory):
                pass
            else:
                raise
    else:
        if overwrite:
            try:
                shutil.rmtree(directory)
            except Exception:
                print('Error while removing directory {}'.format(directory))


class DroneOptimizer(object):
    """ACADOS OCP wrapper for drone MPC with per-rotor thrust control.

    Optimizes over per-rotor normalized inputs [0,1] with full Euler dynamics.
    compute_control_once returns [thrust_N, wx, wy, wz] for attitude_rate mode.

    Args:
        d_model: DroneModel instance (contains model, constraint and all physical params)
        t_horizon: MPC time horizon in seconds
        n_nodes: number of discretization nodes
        enable_logging: enable solver logging
    """

    @staticmethod
    def normalize_quat(q: np.ndarray) -> np.ndarray:
        norm = np.linalg.norm(q)
        if norm > 1e-12:
            return q / norm
        return np.array([1.0, 0.0, 0.0, 0.0])

    def __init__(self, d_model, t_horizon, n_nodes, enable_logging=True):
        # Extract acados model and constraints from DroneModel instance
        model = d_model.model
        d_constraint = d_model.constraint

        self.T = t_horizon
        self.N = n_nodes
        self.enable_logging = enable_logging
        self.log_failures = True

        # Physical parameters from DroneModel
        self.m = d_model.m
        self.g = d_model.g
        self.hover_normalized = d_model.hover_normalized

        # Mixing matrix for converting per-rotor output -> [thrust_N, Mx, My, Mz]
        self.B_mix = d_model.B_mix

        os.chdir(os.path.dirname(os.path.realpath(__file__)))
        self.acados_models_dir = './acados_models'
        safe_mkdir_recursive(self.acados_models_dir)

        acados_source_path = os.environ.get('ACADOS_SOURCE_DIR', '')
        if acados_source_path == '':
            if self.enable_logging:
                print("Warning: ACADOS_SOURCE_DIR not set. Ensure acados_template import works.")
        else:
            if acados_source_path not in sys.path:
                sys.path.insert(0, acados_source_path)

        nx = int(model.x.size()[0])
        self.nx = nx
        nu = int(model.u.size()[0])
        self.nu = nu

        ocp = AcadosOcp()
        ocp.model = model
        ocp.solver_options.N_horizon = self.N
        ocp.solver_options.tf = self.T

        # ============================================
        # External nonlinear cost
        # ============================================
        ocp.cost.cost_type = 'EXTERNAL'
        ocp.cost.cost_type_e = 'EXTERNAL'

        x = model.x
        u = model.u
        x_ref_param = ca.MX.sym('x_ref', nx)

        # Stage cost weights
        Q_pos = 5.0
        Q_vel = 2.0    # increased for velocity damping (reduces hover oscillations)
        Q_quat = 0.5
        Q_omega = 0.2  # increased for angular rate damping
        R_rotor = 0.01  # penalty on deviation from hover per rotor

        # Terminal weights
        Q_pos_term = 5.0
        Q_vel_term = 2.0   # increased for velocity damping
        Q_quat_term = 0.2
        Q_omega_term = 0.05

        # Position error cost
        pos_error = x[0:3] - x_ref_param[0:3]
        pos_cost = Q_pos * ca.dot(pos_error, pos_error)
        pos_cost_term = Q_pos_term * ca.dot(pos_error, pos_error)

        # Velocity error cost
        vel_error = x[3:6] - x_ref_param[3:6]
        vel_cost = Q_vel * ca.dot(vel_error, vel_error)
        vel_cost_term = Q_vel_term * ca.dot(vel_error, vel_error)

        # Quaternion tracking cost. Use a sign-invariant metric because q and -q
        # represent the same attitude.
        q_current = x[6:10]
        q_ref = x_ref_param[6:10]
        q_current_norm = q_current / ca.sqrt(ca.dot(q_current, q_current) + 1e-12)
        q_ref_norm = q_ref / ca.sqrt(ca.dot(q_ref, q_ref) + 1e-12)
        quat_alignment_error = 1.0 - ca.dot(q_current_norm, q_ref_norm) ** 2
        quat_cost = Q_quat * quat_alignment_error
        quat_cost_term = Q_quat_term * quat_alignment_error

        # Angular rate tracking cost
        w_error = x[10:13] - x_ref_param[10:13]
        w_cost = Q_omega * ca.dot(w_error, w_error)
        w_cost_term = Q_omega_term * ca.dot(w_error, w_error)

        # Control cost: penalize deviation from hover (all rotors at hover_normalized)
        u_hover_ca = ca.vertcat(self.hover_normalized, self.hover_normalized,
                                self.hover_normalized, self.hover_normalized)
        u_error = u - u_hover_ca
        control_cost = R_rotor * ca.dot(u_error, u_error)

        # Stage cost
        stage_cost = pos_cost + vel_cost + quat_cost + w_cost + control_cost
        ocp.model.cost_expr_ext_cost = stage_cost

        # Terminal cost (no control penalty)
        terminal_cost = pos_cost_term + vel_cost_term + quat_cost_term + w_cost_term
        ocp.model.cost_expr_ext_cost_e = terminal_cost

        # ============================================
        # Parameters (reference trajectory)
        # ============================================
        ocp.dims.np = nx
        ocp.model.p = x_ref_param
        ocp.parameter_values = np.zeros(nx)

        # ============================================
        # Constraints
        # ============================================
        # Per-rotor thrust constraints [0, 1]
        lbu = d_constraint.u_min
        ubu = d_constraint.u_max
        ocp.constraints.lbu = lbu
        ocp.constraints.ubu = ubu
        ocp.constraints.idxbu = np.array(range(nu))

        # State constraints on angular rates (indices 10, 11, 12)
        ocp.constraints.lbx = d_constraint.w_state_min
        ocp.constraints.ubx = d_constraint.w_state_max
        ocp.constraints.idxbx = np.array([10, 11, 12])

        # Initial condition (will be set at each solve)
        x_init = np.zeros(nx)
        if nx >= 10:
            x_init[6] = 1.0  # identity quaternion (w=1)
        ocp.constraints.x0 = x_init

        # ============================================
        # Solver options
        # ============================================
        ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
        ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
        ocp.solver_options.integrator_type = 'ERK'
        ocp.solver_options.sim_method_num_stages = 4  # RK4 for stiffer Euler dynamics
        ocp.solver_options.sim_method_num_steps = 2
        ocp.solver_options.print_level = 0
        ocp.solver_options.nlp_solver_type = 'SQP'

        # Tolerances
        ocp.solver_options.nlp_solver_tol_eq = 1e-3
        ocp.solver_options.nlp_solver_tol_ineq = 1e-3
        ocp.solver_options.nlp_solver_tol_stat = 1e-2
        ocp.solver_options.nlp_solver_tol_comp = 1e-3

        # Iteration limits
        ocp.solver_options.qp_solver_iter_max = 100
        ocp.solver_options.nlp_solver_max_iter = 50

        # Regularization
        ocp.solver_options.regularize_method = 'CONVEXIFY'
        ocp.solver_options.levenberg_marquardt = 1e-1

        # Globalization
        if hasattr(ocp.solver_options, 'globalization'):
            ocp.solver_options.globalization = 'MERIT_BACKTRACKING'
            if hasattr(ocp.solver_options, 'alpha_min'):
                ocp.solver_options.alpha_min = 0.05
            if hasattr(ocp.solver_options, 'alpha_reduction'):
                ocp.solver_options.alpha_reduction = 0.7

        if hasattr(ocp.solver_options, 'qp_solver_warm_start'):
            ocp.solver_options.qp_solver_warm_start = 1
        if hasattr(ocp.solver_options, 'qp_solver_cond_ric_alg'):
            ocp.solver_options.qp_solver_cond_ric_alg = 0

        # Generate and compile solver
        json_file = os.path.join('./' + model.name + '_acados_ocp.json')
        self.solver = AcadosOcpSolver(ocp, json_file=json_file)
        self.integrator = AcadosSimSolver(ocp, json_file=json_file)

        self.last_x_pred = None
        self.last_u_pred = None
        self.warm_start_available = False
        # Shifted input buffer: u_buffer[i] = u_{i+1}, x_buffer[i] = x_{i+2}
        self._u_buffer = None  # shape (N-1, nu)
        self._x_buffer = None  # shape (N-1, nx)

        if self.enable_logging:
            print(f"Acados OCP solver initialized: N={self.N}, T={self.T}")
            print(f"  State dim: {nx}, Control dim: {nu}")
            print(f"  Control: per-rotor normalized [0, 1]")
            print(f"  Output: [thrust_N, wx, wy, wz] (attitude_rate compatible)")
            print(f"  Hover per-rotor: {self.hover_normalized:.4f}")
            print(f"  Per-rotor constraints: [{lbu[0]:.1f}, {ubu[0]:.1f}]")

    def initialize_solver(self, x_current: np.ndarray, x_ref: np.ndarray, trajectory: bool = False):
        """
        x_current and x_ref are already normalized.
        """
        if trajectory:
            for i in range(self.N + 1):
                self.solver.set(i, 'p', x_ref[:, i])
        else:
            for i in range(self.N + 1):
                self.solver.set(i, 'p', x_ref)

        # Set initial condition
        self.solver.set(0, 'lbx', x_current)
        self.solver.set(0, 'ubx', x_current)

        initialized = False
        if self.warm_start_available:
            try:
                for i in range(min(self.N, self.last_x_pred.shape[0] - 1)):
                    self.solver.set(i, 'x', self.last_x_pred[i + 1, :])
                if self.last_x_pred.shape[0] > 1:
                    self.solver.set(self.N, 'x', self.last_x_pred[-1, :])

                for i in range(min(self.N - 1, self.last_u_pred.shape[0] - 1)):
                    self.solver.set(i, 'u', self.last_u_pred[i + 1, :])
                if self.last_u_pred.shape[0] > 0:
                    self.solver.set(self.N - 1, 'u', self.last_u_pred[-1, :])

                initialized = True
            except Exception as e:
                if self.enable_logging:
                    print(f"Warning: warm start failed: {e}")

        if not initialized:
            if trajectory:
                for i in range(self.N + 1):
                    self.solver.set(i, 'x', x_ref[:, i])
            else:
                for i in range(self.N + 1):
                    alpha = i / max(self.N, 1)
                    x_init = (1 - alpha) * x_current + alpha * x_ref
                    x_init[6:10] = DroneOptimizer.normalize_quat(x_init[6:10])
                    self.solver.set(i, 'x', x_init)

            u_hover = np.array([self.hover_normalized] * 4)
            for i in range(self.N):
                self.solver.set(i, 'u', u_hover)

    def store_predicted_trajectories(self):
        x_pred_all = np.zeros((self.N + 1, self.nx))
        for i in range(self.N + 1):
            x_pred_all[i, :] = self.solver.get(i, 'x').copy()

        u_pred_all = np.zeros((self.N, self.nu))
        for i in range(self.N):
            u_pred_all[i, :] = self.solver.get(i, 'u').copy()

        for k in range(x_pred_all.shape[0]):
            x_pred_all[k, 6:10] = DroneOptimizer.normalize_quat(x_pred_all[k, 6:10])

        self.last_x_pred = x_pred_all.copy()
        self.last_u_pred = u_pred_all.copy()

        if np.isfinite(x_pred_all).all() and np.isfinite(u_pred_all).all():
            self.warm_start_available = True
            # Save u[1..N-1] and x[2..N] as the fallback buffer for the next step.
            # _u_buffer[i] = u_{i+1}, _x_buffer[i] = x_{i+2} (state after u_{i+1})
            self._u_buffer = u_pred_all[1:].copy()
            self._x_buffer = x_pred_all[2:].copy()

    def rotor_to_attitude_cmd(self, u_rotor: np.ndarray, x_next: np.ndarray) -> np.ndarray:
        """
        Convert optimal per-rotor solution to [thrust_N, wx, wy, wz].

        thrust_N: total thrust from mixing matrix (B_mix row 0)
        wx, wy, wz: body rates read from the predicted next state x_next[10:13],
                    which are the result of integrating the full Euler equations.
        """
        thrust_N = float(self.B_mix[0] @ u_rotor)

        # Body rates from predicted next state (integrated by OCP Euler dynamics)
        wx = float(x_next[10])
        wy = float(x_next[11])
        wz = float(x_next[12])

        return np.array([thrust_N, wx, wy, wz])

    def compute_control_once(self, x_current: np.ndarray, x_ref: np.ndarray, is_trajectory: bool = False):
        """
        Solve OCP for one step from current state.

        Args:
            x_current: current state [p(3), v(3), q(4), w(3)]
            x_ref: reference state (same dimensions) or trajectory (nx, N+1)

        Returns:
            u0_attitude: [thrust_N, wx, wy, wz] — thrust in N, rates in rad/s
            x_next: optimal next state
            x_pred_all: predicted state trajectory (N+1, nx)
            u_pred_all: predicted per-rotor trajectory (N, 4)
        """
        x_current[6:10] = DroneOptimizer.normalize_quat(x_current[6:10])
        if not is_trajectory:
            x_ref[6:10] = DroneOptimizer.normalize_quat(x_ref[6:10])
        else:
            for i in range(self.N + 1):
                x_ref[6:10, i] = DroneOptimizer.normalize_quat(x_ref[6:10, i])

        self.initialize_solver(x_current, x_ref, is_trajectory)

        status = self.solver.solve()
        self.warm_start_available = False

        if self.enable_logging:
            residuals = self.solver.get_residuals()
            print(f"[ACADOS] status: {status}, residuals: "
                  f"stat={residuals[0]:.2e}, "
                  f"eq={residuals[1]:.2e}, "
                  f"ineq={residuals[2]:.2e}, "
                  f"comp={residuals[3]:.2e}")

        if status not in [0, 2]:
            print(f"[WARNING] ACADOS solver status {status}")
            if self.log_failures:
                self.log_solver_failures(x_current, x_ref if not is_trajectory else x_ref[:, -1])
            if self._u_buffer is not None and len(self._u_buffer) > 0:
                u_fallback = self.rotor_to_attitude_cmd(self._u_buffer[0], self._x_buffer[0])
                print(f"[WARNING] Using buffered input ({len(self._u_buffer)} steps remaining)")
                self._u_buffer = self._u_buffer[1:]
                self._x_buffer = self._x_buffer[1:]
                return u_fallback, None, self.last_x_pred, self.last_u_pred
            raise Exception(f"acados ocp solver returned status {status}")

        u_rotor = self.solver.get(0, 'u').copy()   # per-rotor [0,1] x4
        x_next = self.solver.get(1, 'x').copy()

        self.store_predicted_trajectories()

        # Convert per-rotor optimal input -> [thrust_N, wx, wy, wz]
        u0_attitude = self.rotor_to_attitude_cmd(u_rotor, x_next)

        return u0_attitude, x_next, self.last_x_pred, self.last_u_pred

    def log_solver_failures(self, x_current: np.ndarray, x_ref: np.ndarray):
        safe_mkdir_recursive('./logs', overwrite=False)
        try:
            np.savetxt('./logs/debug_x0.csv', x_current.reshape(1, -1), delimiter=',')
            np.savetxt('./logs/debug_xref.csv', x_ref.reshape(1, -1), delimiter=',')
            residuals = self.solver.get_residuals()
            np.savetxt('./logs/debug_residuals.csv', residuals.reshape(1, -1), delimiter=',')
        except Exception as e:
            if self.enable_logging:
                print("Error saving debug logs:", e)
