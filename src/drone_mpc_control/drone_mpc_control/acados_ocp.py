import os
import timeit
from typing import Optional, Tuple

import numpy as np
import scipy.linalg

from .drone_model import DroneModel
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver


class AcadosController:
    """Encapsulates ACADOS OCP creation, compilation and solving."""

    def __init__(self, t_horizon: float, n_nodes: int, package_dir: Optional[str] = None,
                 weights=None):
        self.T = t_horizon
        self.N = n_nodes
        self.package_dir = package_dir or os.path.join(
            os.path.expanduser('~'), 'drone', 'drone_mpc_ros2', 'src', 'drone_mpc_control')

        # ensure output dir exists
        self.acados_models_dir = os.path.join(self.package_dir, 'c_generated_code')
        os.makedirs(self.acados_models_dir, exist_ok=True)

        # weights dict expected with keys: position, velocity, quaternion, angular_velocity, control
        self.weights = weights or {}

        # placeholders populated in setup
        self.solver = None
        self.integrator = None
        self.nx = None
        self.nu = None

        # Build/setup immediately
        self.setup_acados()

    def setup_acados(self):
        drone_model_obj = DroneModel()
        model = drone_model_obj.model
        constraint = drone_model_obj.constraint

        # ACADOS installation path
        acados_source_path = os.environ.get('ACADOS_SOURCE_DIR', os.path.expanduser('~/acados'))

        nx = model.x.size()[0]
        nu = model.u.size()[0]
        ny = nx + nu
        n_params = len(model.p)

        self.nx = nx
        self.nu = nu

        # default weights fallback
        w_pos = self.weights.get('position', [200.0, 200.0, 500.0])
        w_vel = self.weights.get('velocity', [1.0, 1.0, 1.0])
        w_quat = self.weights.get('quaternion', [5.0, 5.0, 200.0, 1.0])
        w_ang = self.weights.get('angular_velocity', [1.0, 1.0, 1.0])
        w_ctrl = self.weights.get('control', [6.0, 6.0, 6.0, 6.0])

        ocp = AcadosOcp()
        ocp.acados_include_path = acados_source_path + '/include'
        ocp.acados_lib_path = acados_source_path + '/lib'
        ocp.model = model
        ocp.dims.N = self.N
        ocp.solver_options.tf = self.T

        ocp.dims.np = n_params
        ocp.parameter_values = np.zeros(n_params)

        Q = np.diag(w_pos + w_vel + w_quat + w_ang)
        R = np.diag(w_ctrl)

        ocp.cost.cost_type = 'LINEAR_LS'
        ocp.cost.cost_type_e = 'LINEAR_LS'
        ocp.cost.W = scipy.linalg.block_diag(Q, R)
        ocp.cost.W_e = Q

        ocp.cost.Vx = np.zeros((ny, nx))
        ocp.cost.Vx[:nx, :nx] = np.eye(nx)
        ocp.cost.Vx_e = np.eye(nx)
        ocp.cost.Vu = np.zeros((ny, nu))
        ocp.cost.Vu[nx:, :] = np.eye(nu)

        ocp.constraints.lbu = np.concatenate((
            np.array([constraint.T_min]),
            constraint.M_min
        ))
        ocp.constraints.ubu = np.concatenate((
            np.array([constraint.T_max]),
            constraint.M_max
        ))
        ocp.constraints.idxbu = np.array(range(nu))

        ocp.constraints.lbx = constraint.w_min
        ocp.constraints.ubx = constraint.w_max
        ocp.constraints.idxbx = np.array([10, 11, 12])

        x_init = np.zeros(nx)
        x_init[6] = 1.0
        ocp.constraints.x0 = x_init

        x_ref = np.zeros(nx)
        x_ref[6] = 1.0
        u_ref = np.zeros(nu)
        ocp.cost.yref = np.concatenate((x_ref, u_ref))
        ocp.cost.yref_e = x_ref

        ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
        ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
        ocp.solver_options.integrator_type = 'ERK'
        ocp.solver_options.print_level = 0
        ocp.solver_options.nlp_solver_type = 'SQP_RTI'

        json_file = os.path.join(self.package_dir, model.name + '_acados_ocp.json')
        self.solver = AcadosOcpSolver(ocp, json_file=json_file)
        self.integrator = AcadosSimSolver(ocp, json_file=json_file)

    def solve(self, x_current: np.ndarray, target_state: np.ndarray) -> Tuple[Optional[np.ndarray], float]:
        if not np.isfinite(x_current).all():
            return None, 0.0

        # set x0
        self.solver.set(0, 'lbx', x_current)
        self.solver.set(0, 'ubx', x_current)

        u_ref = np.zeros(self.nu)
        y_ref = np.concatenate((target_state, u_ref))

        for i in range(self.N):
            self.solver.set(i, 'yref', y_ref)
        self.solver.set(self.N, 'yref', target_state)

        start = timeit.default_timer()
        status = self.solver.solve()
        solve_time = timeit.default_timer() - start

        if status != 0:
            return None, solve_time

        u_optimal = self.solver.get(0, 'u')
        return u_optimal, solve_time
