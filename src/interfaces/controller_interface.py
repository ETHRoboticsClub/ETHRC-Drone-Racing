from abc import ABC, abstractmethod
from mavsdk.offboard import AttitudeRate
import numpy as np


class ControllerInterface(ABC):
    # TODO: src/drone_control/drone_control/drone_controller_node accesses
    #       attributes which are unique to MPC in the function sample_trajectory.
    #       This class is hence not yet a properly usable interface.
    @abstractmethod
    def initialize(self):
        '''Builds the MPC optimizer / RL policy'''
        pass

    @abstractmethod
    def trajectory_controller_update(self, x_current :np.ndarray, goal_trajectory :np.ndarray) -> AttitudeRate:
        '''Given the current state and target trajectory, computes the control command'''
        pass

    @abstractmethod
    def controller_update(self, x_current :np.ndarray, goal_position :np.ndarray) -> AttitudeRate:
        '''Given the current state and goal position, computes the control command'''
        pass
