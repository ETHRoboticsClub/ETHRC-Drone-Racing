from abc import ABC, abstractmethod
import numpy as np


class ControllerInterface(ABC):
    """
    Abstract base class for all drone controllers.

    To add a new controller:
      1. Subclass ControllerInterface.
      2. Implement initialize() and controller_update().
      3. If the controller needs waypoints at init (e.g. path-following),
         set needs_waypoints = True and override from_config().
      4. Add the new class to _CONTROLLER_REGISTRY in drone_controller_node.py.
    """

    needs_waypoints: bool = False
    """
    Set to True if this controller requires a waypoints array at instantiation.
    When True, from_config() must read config['waypoints'] (np.ndarray, shape (N, 3)).
    """

    @classmethod
    def from_config(cls, config: dict) -> 'ControllerInterface':
        """
        Instantiate the controller from a config dict.

        The config dict always contains:
          - 'waypoints': np.ndarray (N, 3) or None

        The default implementation calls cls() with no arguments, which is
        correct for controllers that do not need waypoints at construction.
        Override in subclasses that require config values (e.g. MPCC).
        """
        return cls()

    @abstractmethod
    def initialize(self):
        """
        Build the optimizer / load the policy.
        Called once before the control loop starts. May be slow.
        """
        pass

    @abstractmethod
    def controller_update(self, x_current: np.ndarray, goal_position: np.ndarray):
        """
        Compute the next control command.

        Args:
            x_current:     13-dim drone state [p(3), v(3), q(4), ω(3)].
            goal_position: 13-dim goal state  [p(3), v(3), q(4), ω(3)].

        Returns one of:
            AttitudeRate    — for attitude-rate controllers (MPC, MPCC)
            PositionNedYaw  — for position-based controllers (position_ned)
            np.ndarray (4,) — for direct per-rotor thrust controllers (mpc_rotor)

        The drone_controller_node dispatches publishing based on the return type.
        """
        pass
