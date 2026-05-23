from pathlib import Path

from mavsdk.offboard import AttitudeRate
from interfaces import ControllerInterface

from rl.hovering_experiment import HoveringExperiment

_POLICY_PATH = Path(__file__).resolve().parent / "models" / "policy.pt"


class DroneRLController(ControllerInterface):
    def __init__(self):
        super().__init__()
        self.experiment = HoveringExperiment(str(_POLICY_PATH))
        self.velocity_limit = 15.0

    def initialize(self):
        pass  # policy loaded in HoveringExperiment.__init__

    def controller_update(self, x_current, goal_position) -> AttitudeRate:
        cmd = self.experiment.predict_commands(x_current, goal_position)
        
        # Scale the attitude rate commands based on the rate_error_scale parameter.
        # It defaults to 300.0 (scale factor 1.0). Lowering it scales down the rates.
        ros_scale = getattr(self, "rate_error_scale", 300.0)
        scale_factor = ros_scale / 300.0
        
        scaled_roll = float(cmd[0]) * scale_factor
        scaled_pitch = float(cmd[1]) * scale_factor
        scaled_yaw = float(cmd[2]) * scale_factor
        
        return AttitudeRate(scaled_roll, scaled_pitch, scaled_yaw, float(cmd[3]))

    def trajectory_controller_update(self, x_current, goal_trajectory) -> AttitudeRate:
        # Use the current target state from the trajectory (first node in the horizon)
        goal_position = goal_trajectory[:, 0]
        return self.controller_update(x_current, goal_position)

