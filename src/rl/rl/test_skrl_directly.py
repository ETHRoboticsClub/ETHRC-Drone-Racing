import torch
import numpy as np
import os
import sys

# Import SKRL PPO agent and helpers
from skrl.agents.torch.ppo import PPO, PPO_DEFAULT_CONFIG
from skrl.resources.preprocessors.torch import RunningStandardScaler
from skrl.models.torch import Model, GaussianMixin

class Policy(GaussianMixin, Model):
    def __init__(self, observation_space, action_space, device, clip_actions=False, clip_log_std=True, min_log_std=-20, max_log_std=2, initial_log_std=1):
        Model.__init__(self, observation_space, action_space, device)
        GaussianMixin.__init__(self, clip_actions, clip_log_std, min_log_std, max_log_std, initial_log_std)
        
        # Build MLP: 12 -> 256 -> 256 -> 256 -> 4
        self.net = torch.nn.Sequential(
            torch.nn.Linear(observation_space.shape[0], 256),
            torch.nn.ELU(),
            torch.nn.Linear(256, 256),
            torch.nn.ELU(),
            torch.nn.Linear(256, 256),
            torch.nn.ELU(),
            torch.nn.Linear(256, action_space.shape[0])
        )
        self.log_std_parameter = torch.nn.Parameter(torch.full((action_space.shape[0],), initial_log_std))

    def act(self, inputs, role):
        return self.net(inputs["states"]), self.log_std_parameter, {}

# Set up dummy spaces
class DummySpace:
    def __init__(self, shape):
        self.shape = shape

observation_space = DummySpace((12,))
action_space = DummySpace((4,))
device = torch.device("cpu")

# Load checkpoint
checkpoint_path = "/workspace/src/rl/rl/models/policy.pt"
checkpoint = torch.load(checkpoint_path, map_location=device)

# Instantiate policy and agent
policy = Policy(observation_space, action_space, device)
agent_config = PPO_DEFAULT_CONFIG.copy()
agent_config["state_preprocessor"] = RunningStandardScaler
agent_config["state_preprocessor_kwargs"] = {"size": 12, "device": device}

agent = PPO(
    models={"policy": policy},
    memory=None,
    observation_space=observation_space,
    action_space=action_space,
    device=device,
    cfg=agent_config
)

# Load state dict
agent.load(checkpoint_path)
agent.set_running_mode("eval")

# Define cases (same as offline test)
cases = {
    "east": np.array([0, 0, -1, 0, 0, 0, -0.1, 0, 0, 0, 0, 0.5], dtype=np.float32),
    "west": np.array([0, 0, -1, 0, 0, 0, 0.1, 0, 0, 0, 0, 0.5], dtype=np.float32),
    "north": np.array([0, 0, -1, 0, 0, 0, 0, -0.1, 0, 0, 0, 0, 0.5], dtype=np.float32),
    "south": np.array([0, 0, -1, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0.5], dtype=np.float32),
    "on-target": np.array([0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0.5], dtype=np.float32)
}

print("SKRL Directly Loaded Policy Outputs:")
for name, obs in cases.items():
    obs_t = torch.tensor(obs).unsqueeze(0).to(device)
    with torch.no_grad():
        # SKRL agent's act handles preprocessing and passes to model
        # For evaluation, we want the mean action
        act, _, _ = agent.act(obs_t, timestep=0, timesteps=0)
        # Convert actions [-1, 1] to deg/s commands
        raw = act.squeeze(0).numpy().clip(-1.0, 1.0)
        thrust = min((raw[0] + 1.0) * 0.5, 0.6)
        # Convert to deg/s
        omega = raw[1:4] * 9.0 * 180.0 / np.pi
        print(f"{name:10} | roll: {omega[0]:7.2f} | pitch: {-omega[1]:7.2f} | yaw: {-omega[2]:7.2f} | thrust: {thrust:.4f}")
