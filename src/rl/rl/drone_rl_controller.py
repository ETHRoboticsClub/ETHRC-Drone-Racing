from mavsdk.offboard import AttitudeRate
import numpy as np
import torch
import os
from scipy.spatial.transform import Rotation as R
from ament_index_python.packages import get_package_share_directory
from interfaces import ControllerInterface

from rl.policy_model import PolicyModel

class DroneRLController(ControllerInterface):
    def __init__(self):
        super().__init__()
        self.policy_model = None
        self.obs_history = None
        self.last_action = np.zeros(4, dtype=np.float32)
        self.velocity_limit = 15.0 # default from RacingQuadcopterEnvCfg
        self.rate_error_scale = 300.0 # Default value, can be overridden by node parameter

    def initialize(self):
        package_share_directory = get_package_share_directory('rl')
        model_path = os.path.join(package_share_directory, 'models', 'policy.pt')
        self.policy_model = PolicyModel(model_path)

    def controller_update(self, x_current: np.ndarray, goal_position: np.ndarray) -> AttitudeRate:
        # Extract state components
        drone_pos = x_current[0:3]
        lin_vel = x_current[3:6]  # linear velocity (world or body? assuming body per previous impl)
        # Reorder quaternion to [x, y, z, w] for scipy
        quat_drone = np.array([x_current[7], x_current[8], x_current[9], x_current[6]])
        ang_vel = x_current[10:13]  # angular velocity
        
        rot_drone = R.from_quat(quat_drone)
        
        # Compute gravity vector in body frame
        world_gravity = np.array([0.0, 0.0, -9.81])
        gravity_body = rot_drone.inv().apply(world_gravity)
        
        # --- Extract Gate Observations ---
        # The RL env expects 3 lookahead gates. Since we only receive 1 goal_position (13-dim state),
        # we will use it for all 3 gates to prevent index out of bounds or crash.
        gate_pos = goal_position[0:3]
        gate_quat = np.array([goal_position[7], goal_position[8], goal_position[9], goal_position[6]])
        rot_gate = R.from_quat(gate_quat)
        
        # Gate relative position in body frame, scaled by 10.0
        offset_w = gate_pos - drone_pos
        offset_b = rot_drone.inv().apply(offset_w)
        offset_b_scaled = offset_b / 10.0
        
        # Gate forward direction (+Y in gate local frame) mapped to body frame
        y_local = np.array([0.0, 1.0, 0.0])
        forward_w = rot_gate.apply(y_local)
        forward_b = rot_drone.inv().apply(forward_w)
        
        # We need 3 gates, so we duplicate this 3 times
        gate_obs_single = np.concatenate([offset_b_scaled, forward_b])
        obs_gates = np.concatenate([gate_obs_single, gate_obs_single, gate_obs_single]) # 18 dims
        
        # --- Assemble 15-dim step observation ---
        # The policy expects a 15-dim observation representing:
        # [lin_vel(3), ang_vel(3), gravity_body(3), offset_b_scaled(3), forward_b(3)]
        step_obs = np.concatenate([
            lin_vel,                    # 3
            ang_vel,                    # 3
            gravity_body,               # 3
            offset_b_scaled,            # 3
            forward_b                   # 3
        ]).astype(np.float32)
        
        # Convert to tensor and run inference
        observation_tensor = torch.tensor(step_obs, dtype=torch.float32).unsqueeze(0)
        actions = self.policy_model.inference(observation_tensor)
        actions = actions.squeeze(0).cpu().numpy()
        
        # Save last action for next step
        self.last_action = actions.copy()
        
        # --- QUICK AND DIRTY FIX: Torques -> Attitude Rates ---
        # The RL policy outputs thrust (idx 0) and torques (idx 1,2,3) in [-1, 1].
        # Betaflight expects Attitude Rates. We "invert" the Betaflight P-controller:
        # Betaflight Torque ~ Kp * (Rate_Cmd - Rate_Current)
        # Therefore: Rate_Cmd = Rate_Current + (RL_Torque * Scale)
        
        # 1. Map thrust from [-1, 1] to [0, 1]
        thrust_cmd = (actions[0] + 1.0) * 0.5
        
        # 2. Map torques to a rate error using the tunable parameter.
        # Current angular velocity is in rad/s, AttitudeRate expects degrees/s
        ang_vel_deg = np.degrees(ang_vel)
        
        roll_rate_cmd  = ang_vel_deg[0] + (actions[1] * self.rate_error_scale)
        pitch_rate_cmd = ang_vel_deg[1] + (actions[2] * self.rate_error_scale)
        yaw_rate_cmd   = ang_vel_deg[2] + (actions[3] * self.rate_error_scale)
        
        attitude_rate = AttitudeRate(roll_rate_cmd, pitch_rate_cmd, yaw_rate_cmd, float(thrust_cmd))
        return attitude_rate

    def trajectory_controller_update(self, x_current :np.ndarray, goal_trajectory :np.ndarray) -> AttitudeRate:
        print("RL trajectory following is not recommended and not implemented.")
        attitude_rate = AttitudeRate(0, 0, 0, 0)
        return attitude_rate
