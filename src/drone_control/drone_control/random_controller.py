import numpy as np
from interfaces import ControllerInterface
from mavsdk.offboard import AttitudeRate

class DroneRandomController(ControllerInterface):
    """
    A drone controller that commands random attitude rates and thrust.
    Used for baseline testing, safety fallback verification, or comparisons.
    """

    needs_waypoints: bool = False
    t_H: float = 1.5
    Nnodes: int = 15

    def initialize(self):
        """Build the optimizer / load the policy (noop for random)."""
        pass

    def controller_update(self, x_current: np.ndarray, goal_position: np.ndarray):
        """
        Compute the next control command.

        Args:
            x_current:     13-dim drone state [p(3), v(3), q(4), ω(3)].
            goal_position: 13-dim goal state  [p(3), v(3), q(4), ω(3)].

        Returns:
            AttitudeRate containing random roll/pitch/yaw rates and thrust.
        """
        # Generate random roll rate, pitch rate, yaw rate (-50 to 50 deg/s)
        roll_deg_s = np.random.uniform(-50.0, 50.0)
        pitch_deg_s = np.random.uniform(-50.0, 50.0)
        yaw_deg_s = np.random.uniform(-50.0, 50.0)
        
        # Generate random thrust (0.1 to 0.9)
        thrust_value = np.random.uniform(0.1, 0.9)
        
        return AttitudeRate(roll_deg_s, pitch_deg_s, yaw_deg_s, thrust_value)

    def trajectory_controller_update(self, x_current: np.ndarray, desired_trajectory: np.ndarray):
        """
        Compute control commands along a desired trajectory.
        Falls back to standard update since random behavior is time-invariant.
        """
        return self.controller_update(x_current, desired_trajectory)
