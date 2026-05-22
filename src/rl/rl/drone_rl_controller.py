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
        return AttitudeRate(float(cmd[0]), float(cmd[1]), float(cmd[2]), float(cmd[3]))

    def trajectory_controller_update(self, x_current, goal_trajectory) -> AttitudeRate:
        print("RL trajectory tracking is not supported.")
        return AttitudeRate(0, 0, 0, 0)
