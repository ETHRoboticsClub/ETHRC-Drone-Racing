from mavsdk.offboard import AttitudeRate
from interfaces import ControllerInterface

from rl.hovering_experiment import HoveringExperiment


class DroneRLController(ControllerInterface):
    def __init__(self):
        super().__init__()
        self.experiment = HoveringExperiment()
        self.velocity_limit = 15.0


    def initialize(self):
        self.experiment.load_policy()

    def controller_update(self, x_current, goal_position) -> AttitudeRate:
        cmd = self.experiment.predict_commands(x_current, goal_position)
        return AttitudeRate(float(cmd[1]), float(cmd[2]), float(cmd[3]), float(cmd[0]))

    def trajectory_controller_update(self, x_current, goal_trajectory) -> AttitudeRate:
        print("RL trajectory following is not recommended and not implemented.")
        return AttitudeRate(0, 0, 0, 0)
