#!/usr/bin/env python
"""
ACADOS OCP solver wrapper for drone MPC with thrust and body rate commands.
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
    """ACADOS OCP wrapper for drone MPC with thrust/body rate control."""

    def __init__(self, d_model, d_constraint, t_horizon, n_nodes, enable_logging=True):
        model = d_model
        self.T = t_horizon
        self.N = n_nodes
        self.enable_logging = enable_logging
        
        # Store physical parameters for reference
        self.m = 2.5372
        self.g = 9.81


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
        ocp.acados_include_path = acados_source_path + '/include' if acados_source_path else ''
        ocp.acados_lib_path = acados_source_path + '/lib' if acados_source_path else ''
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

        # Stage cost weights - heavily reduced for numerical stability
        Q_pos = 5.0        # Position tracking (was 10.0)
        Q_vel = 0.5        # Velocity tracking (was 1.0)
        Q_quat = 0.0       # Quaternion tracking (was 1.0)
        Q_omega = 0.0     # Angular rate tracking (was 0.1)
        R_thrust = 0.0  # Thrust penalty (was 0.001)
        R_rates = 0.0    # Body rate penalty (was 0.01)

        # Terminal weights - slightly higher than stage weights
        Q_pos_term = 5.0       # Terminal position (was 20.0)
        Q_vel_term = 0.5       # Terminal velocity (was 2.0)
        Q_quat_term = 0.2      # Terminal quaternion (was 2.0)
        Q_omega_term = 0.02   # Terminal angular rate (was 0.2)

        # Position error cost
        pos_error = x[0:3] - x_ref_param[0:3]
        pos_cost = Q_pos * ca.dot(pos_error, pos_error)
        pos_cost_term = Q_pos_term * ca.dot(pos_error, pos_error)

        # Velocity error cost
        vel_error = x[3:6] - x_ref_param[3:6]
        vel_cost = Q_vel * ca.dot(vel_error, vel_error)
        vel_cost_term = Q_vel_term * ca.dot(vel_error, vel_error)

        # Quaternion tracking cost - simplified euclidean distance for stability
        q_current = x[6:10]
        q_ref = x_ref_param[6:10]
        quat_error = q_current - q_ref
        quat_cost = Q_quat * ca.dot(quat_error, quat_error)
        quat_cost_term = Q_quat_term * ca.dot(quat_error, quat_error)

        # Angular rate tracking cost
        w_error = x[10:13] - x_ref_param[10:13]
        w_cost = Q_omega * ca.dot(w_error, w_error)
        w_cost_term = Q_omega_term * ca.dot(w_error, w_error)

        # Control cost - penalize deviation from hover thrust and zero body rates
        u_hover = ca.vertcat(self.m * self.g, 0, 0, 0)
        u_error = u - u_hover
        control_cost = R_thrust * u_error[0]**2 + R_rates * ca.dot(u_error[1:4], u_error[1:4])

        # Stage cost (along trajectory)
        stage_cost = pos_cost + vel_cost + quat_cost + w_cost + control_cost
        ocp.model.cost_expr_ext_cost = stage_cost
        
        # Terminal cost (at end of horizon - no control penalty)
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
        
        # Control input constraints (thrust, wx, wy, wz)
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
        # Solver options - maximum robustness
        # ============================================
        ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
        ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
        ocp.solver_options.integrator_type = 'ERK'
        ocp.solver_options.sim_method_num_stages = 2  # Compromise accuracy/stability
        ocp.solver_options.sim_method_num_steps = 2
        ocp.solver_options.print_level = 0
        ocp.solver_options.nlp_solver_type = 'SQP'
        
        # Relaxed tolerances for robustness
        ocp.solver_options.nlp_solver_tol_eq = 1e-3
        ocp.solver_options.nlp_solver_tol_ineq = 1e-3
        ocp.solver_options.nlp_solver_tol_stat = 1e-2    # Relaxed
        ocp.solver_options.nlp_solver_tol_comp = 1e-3
        
        # Moderate iteration limits
        ocp.solver_options.qp_solver_iter_max = 100
        ocp.solver_options.nlp_solver_max_iter = 50
        
        # Strong regularization for numerical stability
        ocp.solver_options.regularize_method = 'CONVEXIFY'
        ocp.solver_options.levenberg_marquardt = 1e-1  # Heavy regularization
        
        # Aggressive globalization
        if hasattr(ocp.solver_options, 'globalization'):
            ocp.solver_options.globalization = 'MERIT_BACKTRACKING'
            if hasattr(ocp.solver_options, 'alpha_min'):
                ocp.solver_options.alpha_min = 0.05
            if hasattr(ocp.solver_options, 'alpha_reduction'):
                ocp.solver_options.alpha_reduction = 0.7

        # QP solver options for robustness
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

        if self.enable_logging:
            print(f"Acados OCP solver initialized: N={self.N}, T={self.T}")
            print(f"  State dim: {nx}, Control dim: {nu}")
            print(f"  Control: thrust [N], body_rates [rad/s]")
            print(f"  Hover thrust: {self.m * self.g:.2f} N")
            print(f"  Constraints: thrust [{lbu[0]:.1f}, {ubu[0]:.1f}] N")
            print(f"              rates  [{lbu[1]:.1f}, {ubu[1]:.1f}] rad/s")

    def compute_control_once(self, x_current, x_ref):
        """
        Solve OCP for one step from current state.
        
        Args:
            x_current: current state [p(3), v(3), q(4), w(3)]
            x_ref: reference state (same dimensions)
            
        Returns:
            u0: optimal control [thrust, wx, wy, wz]
            x_next: predicted next state
            x_pred_all: predicted state trajectory (N+1, nx)
            u_pred_all: predicted control trajectory (N, nu)
        """
        
        # Normalize quaternion in current state
        q_current = x_current[6:10].copy()
        q_norm = np.linalg.norm(q_current)
        if q_norm > 1e-12:
            x_current[6:10] = q_current / q_norm
        else:
            x_current[6:10] = np.array([1.0, 0.0, 0.0, 0.0])  # Default identity
        
        # Normalize quaternion in reference
        q_ref = x_ref[6:10].copy()
        q_norm_ref = np.linalg.norm(q_ref)
        if q_norm_ref > 1e-12:
            x_ref[6:10] = q_ref / q_norm_ref
        else:
            x_ref[6:10] = np.array([1.0, 0.0, 0.0, 0.0])  # Default identity
        
        # Set reference for all stages
        for i in range(self.N + 1):
            self.solver.set(i, 'p', x_ref)

        # Set initial condition
        self.solver.set(0, 'lbx', x_current)
        self.solver.set(0, 'ubx', x_current)

        # Initialize states with linear interpolation to reference
        for i in range(self.N + 1):
            alpha = i / max(self.N, 1)
            x_init = (1 - alpha) * x_current + alpha * x_ref
            # Re-normalize quaternion
            q_init = x_init[6:10]
            q_init_norm = np.linalg.norm(q_init)
            if q_init_norm > 1e-12:
                x_init[6:10] = q_init / q_init_norm
            self.solver.set(i, 'x', x_init)

        # Initialize controls with hover
        u_hover = np.array([self.m * self.g, 0.0, 0.0, 0.0])
        for i in range(self.N):
            self.solver.set(i, 'u', u_hover)

        # Warm start if available (overrides initialization)
        if (self.last_x_pred is not None) and (self.last_u_pred is not None):
            if (np.isfinite(self.last_x_pred).all() and np.isfinite(self.last_u_pred).all()):
                try:
                    # Shift trajectory forward
                    for i in range(min(self.N, self.last_x_pred.shape[0] - 1)):
                        self.solver.set(i, 'x', self.last_x_pred[i + 1, :])
                    # Last state: duplicate final state
                    if self.last_x_pred.shape[0] > 1:
                        self.solver.set(self.N, 'x', self.last_x_pred[-1, :])
                    
                    # Shift controls
                    for i in range(min(self.N - 1, self.last_u_pred.shape[0] - 1)):
                        self.solver.set(i, 'u', self.last_u_pred[i + 1, :])
                    # Last control: duplicate final control
                    if self.last_u_pred.shape[0] > 0:
                        self.solver.set(self.N - 1, 'u', self.last_u_pred[-1, :])
                except Exception as e:
                    if self.enable_logging:
                        print(f"Warning: warm start failed: {e}")

        # Solve OCP
        status = self.solver.solve()

        if self.enable_logging:
            residuals = self.solver.get_residuals()
            print(f"[ACADOS] status: {status}, residuals: "
                  f"stat={residuals[0]:.2e}, "
                  f"eq={residuals[1]:.2e}, "
                  f"ineq={residuals[2]:.2e}, "
                  f"comp={residuals[3]:.2e}")

        # Handle solver status
        if status not in [0, 2]:
            if self.enable_logging:
                print(f"[WARNING] ACADOS solver status {status}")
            
            safe_mkdir_recursive('./logs', overwrite=False)
            try:
                np.savetxt('./logs/debug_x0.csv', x_current.reshape(1, -1), delimiter=',')
                np.savetxt('./logs/debug_xref.csv', x_ref.reshape(1, -1), delimiter=',')
                residuals = self.solver.get_residuals()
                np.savetxt('./logs/debug_residuals.csv', residuals.reshape(1, -1), delimiter=',')
            except Exception as e:
                if self.enable_logging:
                    print("Error saving debug logs:", e)
            
            # Fallback to hover for critical failures
            if status == 4:
                if self.enable_logging:
                    print("[FALLBACK] Using hover control due to QP solver failure")
                u0 = np.array([self.m * self.g, 0.0, 0.0, 0.0])
                x_next = x_current.copy()
                x_pred_all = np.tile(x_current.reshape(1, -1), (self.N + 1, 1))
                u_pred_all = np.tile(u0.reshape(1, -1), (self.N, 1))
                return u0, x_next, x_pred_all, u_pred_all
            elif status not in [0, 2]:
                raise Exception(f"acados ocp solver returned status {status}")

        # Extract optimal control
        u0 = self.solver.get(0, 'u').copy()

        # Extract predicted trajectories
        x_pred_all = np.zeros((self.N + 1, self.nx))
        for i in range(self.N + 1):
            x_pred_all[i, :] = self.solver.get(i, 'x').copy()

        u_pred_all = np.zeros((self.N, self.nu))
        for i in range(self.N):
            u_pred_all[i, :] = self.solver.get(i, 'u').copy()

        # Normalize quaternions in predictions
        if self.nx >= 10:
            for k in range(x_pred_all.shape[0]):
                q = x_pred_all[k, 6:10]
                qnorm = np.linalg.norm(q)
                if qnorm > 1e-12:
                    x_pred_all[k, 6:10] = q / qnorm

        # Store for warm start
        self.last_x_pred = x_pred_all.copy()
        self.last_u_pred = u_pred_all.copy()

        # Simulate one step with integrator
        self.integrator.set('x', x_current)
        self.integrator.set('u', u0)
        status_sim = self.integrator.solve()
        if status_sim != 0:
            if self.enable_logging:
                print(f"Warning: integrator returned status {status_sim}, using x0")
            x_next = x_current.copy()
        else:
            x_next = self.integrator.get('x').copy()
        
        # Normalize quaternion in x_next
        q_next = x_next[6:10]
        q_next_norm = np.linalg.norm(q_next)
        if q_next_norm > 1e-12:
            x_next[6:10] = q_next / q_next_norm

        return u0, x_next, x_pred_all, u_pred_all