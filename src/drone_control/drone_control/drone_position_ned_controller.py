"""
Position NED controller using MAVSDK PositionNedYaw commands.

This controller forwards goal positions directly to PX4's built-in position
controller via MAVSDK offboard set_position_ned. PX4 handles the low-level
position tracking and attitude control.
"""

import numpy as np
from mavsdk.offboard import PositionNedYaw

from interfaces import ControllerInterface


def _quat_to_yaw_deg(q: np.ndarray) -> float:
    """Extract yaw in degrees from quaternion [qw, qx, qy, qz]."""
    qw, qx, qy, qz = q[0], q[1], q[2], q[3]
    yaw_rad = np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy**2 + qz**2))
    return float(np.degrees(yaw_rad))


def _goal_to_position_ned_yaw(goal: np.ndarray) -> PositionNedYaw:
    """Convert 13-dim goal state to PositionNedYaw (north, east, down, yaw_deg)."""
    north_m = float(goal[0])
    east_m = float(goal[1])
    down_m = float(goal[2])
    yaw_deg = _quat_to_yaw_deg(goal[6:10])
    return PositionNedYaw(north_m, east_m, down_m, yaw_deg)


class DronePositionNedController(ControllerInterface):
    """
    Controller that sends MAVSDK PositionNedYaw commands to PX4.

    Uses PX4's built-in position controller for waypoint tracking. No local
    optimization or policy computation; goal positions are passed through directly.

    t_H and Nnodes are provided for compatibility with drone_controller_node's
    sample_trajectory interpolation when following trajectories.
    """

    t_H: float
    Nnodes: int

    def __init__(self, t_H: float = 1.0, Nnodes: int = 10):
        super().__init__()
        self.t_H = t_H
        self.Nnodes = Nnodes

    @classmethod
    def from_config(cls, config):
        """
        Factory method to instantiate DronePositionNedController from config dict.
        
        For position_ned mode, the config is typically empty.
        Uses default horizon and node parameters.
        """
        t_H = config.get('t_horizon', 1.0)
        Nnodes = config.get('n_nodes', 10)
        return cls(t_H=t_H, Nnodes=Nnodes)

    def initialize(self) -> None:
        """No initialization required."""
        pass

    def controller_update(
        self, x_current: np.ndarray, goal_position: np.ndarray
    ) -> PositionNedYaw:
        """
        Return PositionNedYaw command for the goal position.

        Args:
            x_current: Current state [pos(3), vel(3), quat(4), ang_vel(3)]
            goal_position: Target state with position in [0:3], orientation in [6:10]

        Returns:
            PositionNedYaw(north_m, east_m, down_m, yaw_deg)
        """
        return _goal_to_position_ned_yaw(goal_position)

    def trajectory_controller_update(
        self, x_current: np.ndarray, goal_trajectory: np.ndarray
    ) -> PositionNedYaw:
        """
        Return PositionNedYaw for the current trajectory target.

        Uses the last column of goal_trajectory as the target waypoint.

        Args:
            x_current: Current state
            goal_trajectory: Trajectory matrix, shape (13, N). Target is last column.

        Returns:
            PositionNedYaw for the current waypoint
        """
        target = goal_trajectory[:, -1]
        return _goal_to_position_ned_yaw(target)
