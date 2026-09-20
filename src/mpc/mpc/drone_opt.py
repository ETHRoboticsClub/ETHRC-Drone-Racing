#!/usr/bin/env python
"""
ACADOS OCP solver wrapper for drone MPC with thrust and body rate commands.
"""
import os
import sys
import shutil
import errno
from pathlib import Path
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

    """Lambda to normalize quaternions"""
    normalize_quat = lambda q: q / np.linalg.norm(q) if np.linalg.norm(q) > 1e-12 else np.array([1., 0., 0., 0.])

    def __init__(self, d_model, d_constraint, t_horizon, n_nodes,
                 mass, gravity, enable_logging=True):
        model = d_model
        self.T = t_horizon
        self.N = n_nodes
        self.enable_logging = enable_logging
        self.log_failures = True
        
        # Store physical parameters for reference
        self.m = float(mass)
        self.g = float(gravity)


        self.generated_dir = Path(
            os.environ.get("ETHRC_ACADOS_CACHE", "~/.cache/ethrc_mpc")
        ).expanduser()
        self.acados_models_dir = str(self.generated_dir / 'acados_models')
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
        ocp.code_export_directory = str(self.generated_dir / 'c_generated_code')
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
        Q_quat = 0.5       # Quaternion tracking (was 1.0)
        Q_omega = 0.1     # Angular rate tracking (was 0.1)
        R_thrust = 0.01  # Thrust penalty (was 0.001)
        R_rates = 0.1   # Body rate penalty (was 0.01)

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
        json_file = str(self.generated_dir / f'{model.name}_acados_ocp.json')
        self.solver = AcadosOcpSolver(ocp, json_file=json_file)
        self.integrator = AcadosSimSolver(ocp, json_file=json_file)

        self.last_x_pred = None
        self.last_u_pred = None
        self.warm_start_available = False

        if self.enable_logging:
            print(f"Acados OCP solver initialized: N={self.N}, T={self.T}")
            print(f"  State dim: {nx}, Control dim: {nu}")
            print(f"  Control: thrust [N], body_rates [rad/s]")
            print(f"  Hover thrust: {self.m * self.g:.2f} N")
            print(f"  Constraints: thrust [{lbu[0]:.1f}, {ubu[0]:.1f}] N")
            print(f"              rates  [{lbu[1]:.1f}, {ubu[1]:.1f}] rad/s")

    def initialize_solver(self,x_current :np.ndarray, x_ref :np.ndarray, trajectory: bool = False):
        """
        x_current and x_ref are already normalized
        """
        # Set reference for all stages
        if trajectory :
            for i in range(self.N+1):
                self.solver.set(i, 'p', x_ref[:,i])
        else: 
            for i in range(self.N + 1):
                self.solver.set(i, 'p', x_ref)

        # Set initial condition
        self.solver.set(0, 'lbx', x_current)
        self.solver.set(0, 'ubx', x_current)

        #Initialize Solver States and Control
        initialized =False
        if self.warm_start_available:
            try: #TODO: is try needed? 
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
                
                initialized =True
            except Exception as e:
                if self.enable_logging:
                    print(f"Warning: warm start failed: {e}")
        
        if not initialized: #if no warm start available or warms start fails:

            if trajectory: #interpolation of trajectory and 
                for i in range(self.N + 1):
                    self.solver.set(i, 'x', x_ref[:,i])
            else:       
                # Initialize states with linear interpolation to reference
                for i in range(self.N + 1):
                    alpha = i / max(self.N, 1)
                    x_init = (1 - alpha) * x_current + alpha * x_ref
                    x_init[6:10] = DroneOptimizer.normalize_quat(x_init[6:10])
                    self.solver.set(i, 'x', x_init)

            # Initialize controls with hover
            u_hover = np.array([self.m * self.g, 0.0, 0.0, 0.0])
            for i in range(self.N):
                self.solver.set(i, 'u', u_hover)            

    def store_predicted_trajectories(self):
        # Extract predicted trajectories
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

        if (np.isfinite(x_pred_all).all() and np.isfinite(u_pred_all).all()):
            self.warm_start_available = True

    def compute_control_once(self, x_current :np.ndarray, x_ref :np.ndarray, is_trajectory: bool = False):
        """
        Solve OCP for one step from current state.
        
        Args:
            x_current: current state [p(3), v(3), q(4), w(3)]
            x_ref: reference state (same dimensions)
            
        Returns:
            u0: optimal control [thrust, wx, wy, wz]
            x_next: optimal next state
            x_pred_all: predicted state trajectory (N+1, nx)
            u_pred_all: predicted control trajectory (N, nu)
        """
        
        # Normalize quaternion in current state
        x_current[6:10] = DroneOptimizer.normalize_quat(x_current[6:10])
        if not is_trajectory:
            x_ref[6:10]     = DroneOptimizer.normalize_quat(x_ref[6:10])
        else: 
            for i in range(self.N + 1):
                x_ref[6:10,i]     = DroneOptimizer.normalize_quat(x_ref[6:10,i])
        self.initialize_solver(x_current,x_ref,is_trajectory)
        
        status = self.solver.solve()
        self.warm_start_available = False #reset warm start
                
        # Handle solver status:
        if self.enable_logging:
            residuals = self.solver.get_residuals()
            print(f"[ACADOS] status: {status}, residuals: "
                  f"stat={residuals[0]:.2e}, "
                  f"eq={residuals[1]:.2e}, "
                  f"ineq={residuals[2]:.2e}, "
                  f"comp={residuals[3]:.2e}")

        if (status not in [0, 2]):
            print(f"[WARNING] ACADOS solver status {status}")
            if self.log_failures:
                self.log_solver_failures(x_current, x_ref)
            raise Exception(f"acados ocp solver returned status {status}")
                
        # Extract optimal control & optimal next state
        u0 = self.solver.get(0, 'u').copy()
        x_next = self.solver.get(1, 'x').copy()

        self.store_predicted_trajectories()

        # TODO: why return all this?
        return u0, x_next, self.last_x_pred, self.last_u_pred
    
    def log_solver_failures(self, x_current :np.ndarray,x_ref :np.ndarray):
        # Handle solver status
        
        logs_directory = self.generated_dir / 'logs'
        safe_mkdir_recursive(str(logs_directory), overwrite=False)
        try:
            np.savetxt(
                logs_directory / 'debug_x0.csv',
                x_current.reshape(1, -1),
                delimiter=',',
            )
            np.savetxt(
                logs_directory / 'debug_xref.csv',
                x_ref.reshape(1, -1),
                delimiter=',',
            )
            residuals = self.solver.get_residuals()
            np.savetxt(
                logs_directory / 'debug_residuals.csv',
                residuals.reshape(1, -1),
                delimiter=',',
            )
        except Exception as e:
            if self.enable_logging:
                print("Error saving debug logs:", e)
        
