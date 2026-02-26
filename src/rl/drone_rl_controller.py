from mavsdk.offboard import AttitudeRate
import numpy as np
import torch

from interfaces import ControllerInterface
from rl.policy_model import PolicyModel

class DroneRLController(ControllerInterface):
    def __init__(self):
        super().__init__()
        self.policy_model = None

    def initialize(self):
        self.policy_model = PolicyModel("./rl/models/pytorch/policy_weights.pt")

    def controller_update(self, x_current :np.ndarray, goal_positon :np.ndarray) -> AttitudeRate:
        # TODO: Replace dummy observations with the actual state and goal position.
        #       x_current does not match the observation space of the policy model and
        #       must be preprocessed. The observation space is not fully decided yet.
        #       For now it is:
        #           lin_vel (3), ang_vel (3), gravity (3), first_goal_position (3),
        #           second_goal_position (3) = 15 dimensions
        #       If there is only one goal, use it as both first and second.
        #       Every variable is in the drone's body frame. The gravity vector is
        #       resonsible for providing the drone with a sense of orientation.
        
        dummy_observations = torch.zeros(1, 15)

        actions = self.policy_model.inference(dummy_observations)
        actions = actions.squeeze(0)
        print("Predicted actions:", actions)
        # attitude_rate = AttitudeRate(actions[0], actions[1], actions[2], actions[3])
        # TODO: map the predicted actions to control commands properly
        
        # For now, just make the drone lift off
        thrust = 0.8
        attitude_rate = AttitudeRate(0., 0., 0., thrust)
        return attitude_rate

    def trajectory_controller_update(self, x_current :np.ndarray, goal_trajectory :np.ndarray) -> AttitudeRate:
        print("RL trajectory following is not recommended and not implemented.")
        attitude_rate = AttitudeRate(0, 0, 0, 0)
        return attitude_rate
