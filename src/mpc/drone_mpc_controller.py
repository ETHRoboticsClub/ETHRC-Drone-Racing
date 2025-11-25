from .drone_model import DroneModel
from .drone_opt import DroneOptimizer
import numpy as np

from mavsdk.offboard import AttitudeRate

DEBUG_INFO_PRINTS = True

class DroneMpcController():

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

    async def initialize(self):
        """
        Initialize Class
        Constructor should return fast, so async operations are moved here. 
        """
        #TODO: fix, as this builds the model each time,
        self.opt = DroneOptimizer(  d_model=self.drone_model.model,
                            d_constraint=self.drone_model.constraint,
                            t_horizon=self.t_H,
                            n_nodes=self.Nnodes,
                            enable_logging=True)


    def controller_update(self, x_current :np.ndarray, goal_positon :np.ndarray) -> AttitudeRate :
        # TODO: bad, 2 change
        self.x_current = x_current
        self.goal_postion = goal_positon

        # Solve OCP
        try:
            #TODO: bound solution time, ensure RT
            u0 = self.opt.compute_control_once(self.x_current, self.goal_postion)

            if DEBUG_INFO_PRINTS:
                self.mpc_iter += 1

                if self.mpc_iter % 10 == 0: 
                    print(f"[MPC iter {self.mpc_iter}] "
                            f"pos=({self.x_current[0]:.2f},{self.x_current[1]:.2f},{self.x_current[2]:.2f}) "
                            f"goal=({self.goal_postion[0]:.2f},{self.goal_postion[1]:.2f},{self.goal_postion[2]:.2f})")
                    print(f"  u=[T={u0[0]:.2f}N, "
                            f"wx={u0[1]:.3f}, wy={u0[2]:.3f}, wz={u0[3]:.3f}]")
        except Exception as e:
            print(f"[MPC Controller] ERROR: Optimizer failed: {e}")
            return AttitudeRate(0.0, 0.0, 0.0, self.thrust_norm_hover)
        
        # Send Command
        # MAVSDK AttitudeRate: roll_deg_s, pitch_deg_s, yaw_deg_s, thrust_value
        # thrust_value: [0,1] normalized thrust

        thrust_normalized = self._thrust_to_normalized(u0[0])
        
        # Convert body rates from rad/s to deg/s
        roll_rate_deg  = np.degrees(u0[1])
        pitch_rate_deg = np.degrees(u0[2])
        yaw_rate_deg   = np.degrees(u0[3])
        
        # Safety clipping
        max_rate_deg = 200.0  # deg/s
        roll_rate_deg = np.clip(roll_rate_deg, -max_rate_deg, max_rate_deg)
        pitch_rate_deg = np.clip(pitch_rate_deg, -max_rate_deg, max_rate_deg)
        yaw_rate_deg = np.clip(yaw_rate_deg, -max_rate_deg, max_rate_deg)

        return AttitudeRate(roll_rate_deg, pitch_rate_deg, yaw_rate_deg, thrust_normalized)

    def _thrust_to_normalized(self, thrust_N):
        """Convert thrust in Newtons to normalized [0,1] for MAVSDK."""
        return float(np.clip(thrust_N / self.max_thrust, 0.0, 1.0))

    # @staticmethod
    # def compute_thrust_vector_ned(thrust_magnitude, quaternion):
    #     """
    #     Compute thrust vector in NED frame from thrust magnitude and drone orientation.
        
    #     Args:
    #         thrust_magnitude: thrust magnitude in Newtons
    #         quaternion: attitude quaternion [w, x, y, z]
            
    #     Returns:
    #         thrust_vector_ned: (3,) thrust vector in NED frame
    #     """
    #     # Thrust vector in body frame (pointing up = +z_body)
    #     thrust_body = np.array([0.0, 0.0, thrust_magnitude])
        
    #     # Rotation matrix from body to NED using quaternion
    #     qw, qx, qy, qz = quaternion
        
    #     # Rotation matrix body -> NED
    #     R_bn = np.array([
    #         [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy)],
    #         [2*(qx*qy + qw*qz), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qw*qx)],
    #         [2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1 - 2*(qx**2 + qy**2)]
    #     ])
        
    #     # Transform thrust to NED frame
    #     thrust_ned = R_bn @ thrust_body
        
    #     return thrust_ned