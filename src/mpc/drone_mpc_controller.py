from .drone_model import DroneModel
from .drone_opt import DroneOptimizer
import numpy as np

from mavsdk.offboard import AttitudeRate

from interfaces import ControllerInterface

DEBUG_INFO_PRINTS = True

class DroneMpcController(ControllerInterface):

    drone_model : DroneModel
    opt: DroneOptimizer

    t_H: float 
    Nnodes: int

    #dynamic params:
    max_thrust: float
    drone_mass: float
    gravity: float
    thrust_norm_hover: float

    #drone_state
    x_current: np.ndarray
    goal_postion: np.ndarray
    # last_cmd: np.ndarray


    #TODO: implement this
    goal_trajectory_pos: np.ndarray
    goal_trajectory_ts: np.ndarray

    #debug variables
    mpc_iter: int = 0

    def __init__(self, t_H = 1.5, Nnodes = 15):
        self.t_H = t_H
        self.Nnodes = Nnodes

        self.drone_model = DroneModel()

        self.drone_mass = float(self.drone_model.m)
        self.gravity = float(self.drone_model.g)
        self.max_thrust = float(self.drone_model.constraint.thrust_max)
        self.thrust_norm_hover = self.drone_mass * self.gravity / self.max_thrust 

        if DEBUG_INFO_PRINTS:
            print(f"\n{'='*60}")
            print(f"Drone parameters:")
            print(f"  Mass: {self.drone_mass:.2f} kg")
            print(f"  Hover thrust: {self.drone_mass * self.gravity:.2f} N")
            print(f"  Max thrust: {self.max_thrust:.2f} N")
            print(f"{'='*60}\n")

    def initialize(self):
        """
        Initialize Class
        Constructor should return fast, so async operations are moved here. 
        """
        #TODO: fix, as this builds the model each time,
        self.opt = DroneOptimizer(  d_model=self.drone_model,
                            t_horizon=self.t_H,
                            n_nodes=self.Nnodes,
                            enable_logging=True)

    def trajectory_controller_update(self, x_current :np.ndarray, goal_trajectory :np.ndarray):
        # TODO: bad, 2 change
        self.x_current = x_current
        self.goal_postion = goal_trajectory[:,-1]

        # Solve OCP
        # TODO: Handle mpc failure more explicitely
        try:
            #TODO: bound solution time, ensure RT
            u0 = self.opt.compute_control_once(self.x_current, goal_trajectory, is_trajectory=True)

            if DEBUG_INFO_PRINTS:
                self._debug_print(u0)

        except Exception as e:
            print(f"[MPC Controller] ERROR: Optimizer failed: {e}")
            return AttitudeRate(0.0, 0.0, 0.0, self.thrust_norm_hover)
        
        thrust_N = float(u0[0])
        wx_cmd = float(u0[1])  # roll rate
        wy_cmd = float(u0[2])  # pitch rate
        wz_cmd = float(u0[3])  # yaw rate

        return self.attitude_cmd_from_mpc_cmd(thrust_N,wx_cmd,wy_cmd,wz_cmd)


    def controller_update(self, x_current :np.ndarray, goal_position :np.ndarray) -> AttitudeRate :
        # TODO: bad, 2 change
        self.x_current = x_current
        self.goal_position = goal_position

        # Solve OCP
        # TODO: Handle mpc failure more explicitely
        try:
            #TODO: bound solution time, ensure RT
            u0, _, _, _ = self.opt.compute_control_once(self.x_current, self.goal_position)

            if DEBUG_INFO_PRINTS:
                self._debug_print(u0)

        except Exception as e:
            print(f"[MPC Controller] ERROR: Optimizer failed: {e}")
            return AttitudeRate(0.0, 0.0, 0.0, self.thrust_norm_hover)
        
        thrust_N = float(u0[0])
        wx_cmd = float(u0[1])  # roll rate
        wy_cmd = float(u0[2])  # pitch rate
        wz_cmd = float(u0[3])  # yaw rate

        return self.attitude_cmd_from_mpc_cmd(thrust_N,wx_cmd,wy_cmd,wz_cmd)

    def attitude_cmd_from_mpc_cmd(self, thrust_N: float, wx_cmd: float, wy_cmd: float, wz_cmd: float ):
        # Send Command
        # MAVSDK AttitudeRate: roll_deg_s, pitch_deg_s, yaw_deg_s, thrust_value
        # thrust_value: [0,1] normalized thrust
        thrust_normalized = self._thrust_to_normalized(thrust_N)
        
        # Convert body rates from rad/s to deg/s
        roll_rate_deg  = np.degrees(wx_cmd)
        pitch_rate_deg = np.degrees(wy_cmd)
        yaw_rate_deg   = np.degrees(wz_cmd)
        
        # Safety clipping
        max_rate_deg = 200.0  # deg/s
        roll_rate_deg = np.clip(roll_rate_deg, -max_rate_deg, max_rate_deg)
        pitch_rate_deg = np.clip(pitch_rate_deg, -max_rate_deg, max_rate_deg)
        yaw_rate_deg = np.clip(yaw_rate_deg, -max_rate_deg, max_rate_deg)

        return AttitudeRate(roll_rate_deg, pitch_rate_deg, yaw_rate_deg, thrust_normalized)

    def _thrust_to_normalized(self, thrust_N):
        """Convert thrust in Newtons to normalized [0,1] for MAVSDK."""
        return float(np.clip(thrust_N / self.max_thrust, 0.0, 1.0))
    
    def _debug_print(self,u0):
        self.mpc_iter += 1

        if self.mpc_iter % 10 == 0: 
            print(f"[MPC iter {self.mpc_iter}] "
                    f"pos=({self.x_current[0]:.2f},{self.x_current[1]:.2f},{self.x_current[2]:.2f}) "
                    f"goal=({self.goal_postion[0]:.2f},{self.goal_postion[1]:.2f},{self.goal_postion[2]:.2f})")
            # u0 is [thrust_N, wx, wy, wz]
            print(f"  u=[T={float(u0[0]):.2f}N, "
                    f"wx={float(u0[1]):.3f}, wy={float(u0[2]):.3f}, wz={float(u0[3]):.3f}]")


