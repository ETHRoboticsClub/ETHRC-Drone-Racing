from .drone_model_rotor import DroneModelRotor
from .drone_opt_rotor import DroneOptimizerRotor
import numpy as np

DEBUG_INFO_PRINTS = True


class DroneMpcControllerRotor():
    """
    MPC controller wrapper for per-rotor thrust mode.
    Outputs u_rotors[4] normalized [0, 1] for direct actuator control.
    """

    drone_model: DroneModelRotor
    opt: DroneOptimizerRotor

    t_H: float
    Nnodes: int

    # physical params
    hover_normalized: float

    # drone state
    x_current: np.ndarray
    goal_postion: np.ndarray

    # debug
    mpc_iter: int = 0
    consecutive_failures: int = 0
    MAX_CONSECUTIVE_FAILURES: int = 3

    def __init__(self, t_H=1.5, Nnodes=15):
        self.t_H = t_H
        self.Nnodes = Nnodes

        self.drone_model = DroneModelRotor()
        self.hover_normalized = float(self.drone_model.hover_normalized)

        if DEBUG_INFO_PRINTS:
            print(f"\n{'='*60}")
            print(f"Drone Rotor Controller parameters:")
            print(f"  Mass: {self.drone_model.m:.2f} kg")
            print(f"  Hover per-rotor (normalized): {self.hover_normalized:.4f}")
            print(f"  Max single rotor thrust: {self.drone_model.maxSingleRotorThrust:.2f} N")
            print(f"{'='*60}\n")

    def initialize(self):
        """Initialize the optimizer. Constructor returns fast; this does the heavy work."""
        self.opt = DroneOptimizerRotor(
            d_model=self.drone_model,
            t_horizon=self.t_H,
            n_nodes=self.Nnodes,
            hover_normalized=self.hover_normalized,
            enable_logging=True)

    def controller_update(self, x_current: np.ndarray, goal_state: np.ndarray) -> np.ndarray:
        """
        Compute per-rotor commands for a single waypoint goal.

        Args:
            x_current: current state [13]
            goal_state: reference state [13]

        Returns:
            u_rotors: np.ndarray[4] normalized [0, 1] per-rotor commands
        """
        self.x_current = x_current
        self.goal_postion = goal_state

        try:
            u0, x_next, x_pred_all, u_pred_all = self.opt.compute_control_once(
                self.x_current, self.goal_postion)

            self.consecutive_failures = 0

            if DEBUG_INFO_PRINTS:
                self._debug_print(u0)

            # u0 is [u0, u1, u2, u3] already in [0, 1] from solver constraints
            return np.clip(u0, 0.0, 1.0)

        except Exception as e:
            self.consecutive_failures += 1
            print(f"[MPC Rotor Controller] ERROR: Optimizer failed ({self.consecutive_failures}x): {e}")

            # Failsafe: hover on all rotors
            return np.full(4, self.hover_normalized)

    def trajectory_controller_update(self, x_current: np.ndarray,
                                      goal_trajectory: np.ndarray) -> np.ndarray:
        """
        Compute per-rotor commands for trajectory tracking.

        Args:
            x_current: current state [13]
            goal_trajectory: reference trajectory [13, N+1]

        Returns:
            u_rotors: np.ndarray[4] normalized [0, 1]
        """
        self.x_current = x_current
        self.goal_postion = goal_trajectory[:, -1]

        try:
            u0 = self.opt.compute_control_once(
                self.x_current, goal_trajectory, is_trajectory=True)

            self.consecutive_failures = 0

            if DEBUG_INFO_PRINTS:
                self._debug_print(u0)

            return np.clip(u0, 0.0, 1.0)

        except Exception as e:
            self.consecutive_failures += 1
            print(f"[MPC Rotor Controller] ERROR: Optimizer failed ({self.consecutive_failures}x): {e}")
            return np.full(4, self.hover_normalized)

    def get_predicted_trajectories(self):
        """Return last predicted state and control trajectories for visualization."""
        if self.opt.last_x_pred is not None and self.opt.last_u_pred is not None:
            return self.opt.last_x_pred, self.opt.last_u_pred
        return None, None

    def _debug_print(self, u0):
        self.mpc_iter += 1
        if self.mpc_iter % 10 == 0:
            print(f"[MPC-Rotor iter {self.mpc_iter}] "
                  f"pos=({self.x_current[0]:.2f},{self.x_current[1]:.2f},{self.x_current[2]:.2f}) "
                  f"goal=({self.goal_postion[0]:.2f},{self.goal_postion[1]:.2f},{self.goal_postion[2]:.2f})")
            print(f"  u_rotors=[{u0[0]:.4f}, {u0[1]:.4f}, {u0[2]:.4f}, {u0[3]:.4f}]"
                  f"  (hover={self.hover_normalized:.4f})")
