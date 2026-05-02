from mavsdk.offboard import AttitudeRate
import numpy as np
import torch
from scipy.spatial.transform import Rotation as R
from interfaces import ControllerInterface

from rl.policy_model import PolicyModel

class DroneRLController(ControllerInterface):
    def __init__(self):
        super().__init__()
        self.policy_model = None

    def initialize(self):
        self.policy_model = PolicyModel("src/rl/models/pytorch/policy_weights.pt")

    def controller_update(self, x_current :np.ndarray, goal_positon :np.ndarray) -> AttitudeRate:
        # Extract state components
        lin_vel = x_current[3:6]  # linear velocity (body frame)
        ang_vel = x_current[10:13]  # angular velocity
        quat = x_current[6:10]  # quaternion [w, x, y, z]
        
        # Compute gravity vector in body frame
        world_gravity = np.array([0.0, 0.0, -9.81])
        # Reorder quaternion to [x, y, z, w] for scipy
        quat_scipy = np.array([quat[1], quat[2], quat[3], quat[0]])
        rotation = R.from_quat(quat_scipy)
        gravity_body = rotation.inv().apply(world_gravity)
        
        # Goal positions
        first_goal_pos = goal_positon[0:3]
        second_goal_pos = first_goal_pos  # Use same goal for both if only one
        
        # Concatenate into 15-dim observation
        observation = np.concatenate([lin_vel, ang_vel, gravity_body, first_goal_pos, second_goal_pos])
        
        # Convert to tensor and run inference
        observation_tensor = torch.tensor(observation, dtype=torch.float32).unsqueeze(0)
        actions = self.policy_model.inference(observation_tensor)
        actions = actions.squeeze(0).detach().numpy()
        
        # Map actions to AttitudeRate (assuming [roll_deg_s, pitch_deg_s, yaw_deg_s, thrust_value])
        attitude_rate = AttitudeRate(actions[0], actions[1], actions[2], actions[3])
        return attitude_rate

    def trajectory_controller_update(self, x_current :np.ndarray, goal_trajectory :np.ndarray) -> AttitudeRate:
        print("RL trajectory following is not recommended and not implemented.")
        attitude_rate = AttitudeRate(0, 0, 0, 0)
        return attitude_rate
