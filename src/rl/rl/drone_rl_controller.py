import logging
import os
from logging.handlers import RotatingFileHandler
from pathlib import Path

import numpy as np
from mavsdk.offboard import AttitudeRate
from interfaces import ControllerInterface

from rl.hovering_experiment import HoveringExperiment

_DEFAULT_POLICY_PATH = Path(__file__).resolve().parent / "models" / "policy.pt"
# Allow evaluation of candidate checkpoints without replacing the accepted
# deployment artifact. This is intentionally process-scoped for SITL tests.
_POLICY_PATH = Path(os.environ.get("RL_POLICY_PATH", str(_DEFAULT_POLICY_PATH))).expanduser().resolve()
_LOG_PATH = Path.home() / ".ros" / "rl_controller.log"


class DroneRLController(ControllerInterface):
    def __init__(self):
        super().__init__()
        self.experiment = HoveringExperiment(str(_POLICY_PATH))
        self._logger = self._build_logger()
        self._log_every_n = 10  # log 1 in N controller_update calls
        self._update_counter = 0
        self._logger.info(
            "DroneRLController created pid=%d policy=%s",
            os.getpid(), _POLICY_PATH,
        )

    @property
    def last_observation(self):
        return self.experiment.last_observation

    @staticmethod
    def _build_logger() -> logging.Logger:
        logger = logging.getLogger("drone_rl_controller")
        if getattr(logger, "_rl_configured", False):
            return logger
        logger.setLevel(logging.INFO)
        logger.propagate = False
        _LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
        handler = RotatingFileHandler(_LOG_PATH, maxBytes=10 * 1024 * 1024, backupCount=3)
        handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
        logger.addHandler(handler)
        logger._rl_configured = True
        return logger

    def initialize(self):
        pass  # policy loaded in HoveringExperiment.__init__

    def controller_update(self, x_current, goal_position) -> AttitudeRate:
        cmd = self.experiment.predict_commands(x_current, goal_position)


        roll = float(cmd[0])
        pitch = float(cmd[1])
        yaw = float(cmd[2])

        dist_xy = float(np.linalg.norm(goal_position[:2] - x_current[:2]))

        thrust = float(cmd[3])

        # Throttled logging every N updates
        self._update_counter += 1
        if self._update_counter % self._log_every_n == 0:
            self._logger.info(
                "state=%s goal=%s raw_cmd (roll, pitch, yaw, thrust)=[%.4f,%.4f,%.4f,%.4f] "
                "dist_xy=%.4f out=[%.4f,%.4f,%.4f,%.4f]",
                np.array2string(np.asarray(x_current), precision=4, separator=","),
                np.array2string(np.asarray(goal_position), precision=4, separator=","),
                float(cmd[0]), float(cmd[1]), float(cmd[2]), float(cmd[3]),
                dist_xy, roll, pitch, yaw, thrust,
            )
        return AttitudeRate(roll, pitch, yaw, thrust)
