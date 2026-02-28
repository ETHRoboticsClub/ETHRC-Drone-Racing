# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Racing Quadcopter Environment with Gate Terrain Generator.

This environment trains a quadcopter to fly through racing gates using:
- Terrain-generated gates with proper collision meshes
- Gate-based rewards (passage, progress, centering)
- Fully vectorised reward / observation computation (no per-env Python loops)
"""

from __future__ import annotations

import gymnasium as gym
import numpy as np
import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg
from isaaclab.envs.common import ViewerCfg
from isaaclab.envs.ui import BaseEnvWindow
from isaaclab.markers import CUBOID_MARKER_CFG, VisualizationMarkers
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.terrains.config.racing import create_racing_gates_terrain
from isaaclab.terrains.trimesh.racing_gates import get_gate_registry
from isaaclab.utils import configclass
from isaaclab.utils.math import quat_apply, quat_apply_inverse

from isaaclab_assets import CRAZYFLIE_CFG
from .reward_manager import GateRewardManager, RewardWeights

from .reward_manager import GateRewardManager, RewardWeights


# ---------------------------------------------------------------------------
# UI
# ---------------------------------------------------------------------------


class QuadcopterEnvWindow(BaseEnvWindow):
    """Window manager for the Quadcopter environment."""

    def __init__(self, env: "RacingQuadcopterEnv", window_name: str = "IsaacLab"):
        super().__init__(env, window_name)
        with self.ui_window_elements["main_vstack"]:
            with self.ui_window_elements["debug_frame"]:
                with self.ui_window_elements["debug_vstack"]:
                    self._create_debug_vis_ui_element("targets", self.env)


# ---------------------------------------------------------------------------
# Sub-configs
# ---------------------------------------------------------------------------


@configclass
class GateRewardConfig:
    """Configuration for gate-based rewards.
    
    Primary rewards encourage gate passage and proper trajectory.
    Penalties discourage unsafe or inefficient behavior.
    """

    # Primary rewards (positive)
    gate_passage_reward: float = 20.0   # Large reward for passing through gate
    gate_progress_scale: float = 2.0     # Reward for moving toward next gate
    gate_centering_scale: float = 0.5    # Reward for staying aligned with gate center
    velocity_toward_gate_scale: float = 0.1  # Reward for velocity toward gate
    
    # Penalties (negative)
    velocity_limit_penalty_scale: float = -0.05
    orientation_penalty_scale: float = -0.005
    ang_vel_penalty_scale: float = -0.001
    action_smoothness_scale: float = -0.0001
    crash_penalty: float = -50.0
    wrong_side_penalty_scale: float = -0.1

    reward_scale: float = 1/100.0 # Overall scaling to keep rewards in a reasonable range
    
    def to_reward_weights(self) -> RewardWeights:
        """Convert to RewardWeights dataclass."""
        return RewardWeights(
            gate_passage=self.gate_passage_reward,
            gate_progress=self.gate_progress_scale,
            gate_centering=self.gate_centering_scale,
            velocity_toward_gate=self.velocity_toward_gate_scale,
            velocity_limit=self.velocity_limit_penalty_scale,
            orientation=self.orientation_penalty_scale,
            angular_velocity=self.ang_vel_penalty_scale,
            action_smoothness=self.action_smoothness_scale,
            crash=self.crash_penalty,
            wrong_side=self.wrong_side_penalty_scale,
            reward_scale=self.reward_scale,
        )


@configclass
class GateObservationConfig:
    """Configuration for gate-based observations."""

    num_next_gates: int = 5  # Increased from 3 to 5 for longer trajectories
    history_length: int = 0
    include_velocity_limit: bool = True


# ---------------------------------------------------------------------------
# Main environment config
# ---------------------------------------------------------------------------


@configclass
class RacingQuadcopterEnvCfg(DirectRLEnvCfg):
    """Configuration for racing quadcopter environment with terrain-generated gates."""

    # Environment settings
    episode_length_s: float = 20.0
    decimation: int = 2
    action_space: int = 4
    state_space: int = 0
    debug_vis: bool = False

    # Velocity settings
    velocity_limit: float = 15.0
    randomize_velocity_limit: bool = False
    velocity_limit_range: tuple[float, float] = (8.0, 15.0)

    # Reward / observation sub-configs
    reward: GateRewardConfig = GateRewardConfig()
    observation: GateObservationConfig = GateObservationConfig()

    # Observation space (recalculated in __post_init__)
    # lin_vel(3) + ang_vel(3) + gravity(3) + vel_limit(1) + gates(N * 6)
    # Each gate: rel_pos(3) + forward_dir(3)
    observation_space: int = 10 + 5 * 6  # Updated for 5 gates

    # Termination thresholds
    upside_down_threshold: float = 0.7
    min_height: float = 0.2
    max_velocity: float = 50.0

    # UI
    ui_window_class_type = QuadcopterEnvWindow

    # Simulation
    sim: SimulationCfg = SimulationCfg(
        dt=1 / 50,
        render_interval=2,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=1.0,
            dynamic_friction=1.0,
            restitution=0.0,
        ),
    )

    # Terrain with racing gates (configured in __post_init__ based on observation config)
    terrain: TerrainImporterCfg = None

    # Scene
    scene: InteractiveSceneCfg = InteractiveSceneCfg(
        num_envs=1024,
        env_spacing=5.0,  # Increased spacing for larger terrain (15x15m)
        replicate_physics=True,
        clone_in_fabric=True,
    )

    camera_on_drone: bool = False

    if camera_on_drone:
        viewer: ViewerCfg = ViewerCfg(
            eye=(-0.1, 0.0, 0.1), lookat=(5.0, 0.0, -0.5), origin_type="asset_root", env_index=0, asset_name="robot", body_name="body")
    else:
        # Position camera to see the first environment's terrain (15x15m terrain)
        viewer: ViewerCfg = ViewerCfg(
            eye=(25.0, 25.0, 15.0), lookat=(10.0, 10.0, 2.5), origin_type="world")


    # Robot
    # robot: ArticulationCfg = CRAZYFLIE_CFG.replace(
    #     prim_path="/World/envs/env_.*/Robot",
    # )

    robot: ArticulationCfg = CRAZYFLIE_CFG.replace(prim_path="/World/envs/env_.*/Robot").\
                                           replace(spawn = CRAZYFLIE_CFG.spawn.replace(activate_contact_sensors=False))
    thrust_to_weight: float = 1.9
    moment_scale: float = 0.01

    def __post_init__(self):
        """Post initialization."""
        base = 10  # lin_vel(3) + ang_vel(3) + gravity(3) + vel_limit(1)
        gates = self.observation.num_next_gates * 6  # position(3) + forward_direction(3) for each gate
        history = self.observation.history_length * 10 if self.observation.history_length > 0 else 0
        self.observation_space = base + gates + history
        self.sim.render_interval = self.decimation
        
        # Use enhanced configuration for longer, more complex trajectories
        from isaaclab.terrains.config.racing import RACING_GATES_SIMPLE_CFG
        from isaaclab.terrains import terrain_generator as terrain_gen
        print(f"[DEBUG] Creating enhanced racing terrain for longer, more complex trajectories")
        
        # Create enhanced configuration with longer, more challenging trajectories
        terrain_generator = create_racing_gates_terrain()
        
        self.terrain = TerrainImporterCfg(
            prim_path="/World/ground",
            terrain_type="generator",
            terrain_generator=terrain_generator,
            max_init_terrain_level=0,
            collision_group=-1,
            physics_material=sim_utils.RigidBodyMaterialCfg(
                friction_combine_mode="multiply",
                restitution_combine_mode="multiply",
                static_friction=1.0,
                dynamic_friction=1.0,
            ),
            debug_vis=True,
        )


# ---------------------------------------------------------------------------
# Environment
# ---------------------------------------------------------------------------


class RacingQuadcopterEnv(DirectRLEnv):
    """Racing quadcopter environment using terrain-generated gates.

    Gates are generated as part of the terrain mesh by :class:`MeshRacingGatesTerrainCfg`.
    Gate positions/orientations are recovered from a module-level registry
    that the terrain function writes to during generation.

    All reward and observation computations are **fully vectorised** -- no per-env
    Python ``for`` loops on the hot path.
    """

    cfg: RacingQuadcopterEnvCfg

    def __init__(self, cfg: RacingQuadcopterEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)

        # --- action buffers ---
        self._actions = torch.zeros(self.num_envs, gym.spaces.flatdim(self.single_action_space), device=self.device)
        self._previous_actions = torch.zeros_like(self._actions)
        self._thrust = torch.zeros(self.num_envs, 1, 3, device=self.device)
        self._moment = torch.zeros(self.num_envs, 1, 3, device=self.device)

        # --- robot properties ---
        self._body_id = self._robot.find_bodies("body")[0]
        self._robot_mass = self._robot.root_physx_view.get_masses()[0].sum()
        self._gravity_magnitude = torch.tensor(self.sim.cfg.gravity, device=self.device).norm()
        self._robot_weight = (self._robot_mass * self._gravity_magnitude).item()

        # --- velocity limit ---
        self._velocity_limit = torch.full((self.num_envs,), cfg.velocity_limit, device=self.device)

        # --- gate tracking (populated by _setup_gate_tracking) ---
        self._setup_gate_tracking()

        # --- observation history ---
        if cfg.observation.history_length > 0:
            hl = cfg.observation.history_length
            self._lin_vel_history = torch.zeros(self.num_envs, hl, 3, device=self.device)
            self._ang_vel_history = torch.zeros(self.num_envs, hl, 3, device=self.device)
            self._action_history = torch.zeros(self.num_envs, hl, cfg.action_space, device=self.device)

        # --- pre-allocated local-frame direction vectors ---
        self._y_local = torch.tensor([0.0, 1.0, 0.0], device=self.device)
        self._x_local = torch.tensor([1.0, 0.0, 0.0], device=self.device)
        self._z_local = torch.tensor([0.0, 0.0, 1.0], device=self.device)

        # --- reward manager ---
        reward_weights = cfg.reward.to_reward_weights()
        self._reward_manager = GateRewardManager(
            num_envs=self.num_envs,
            device=self.device,
            weights=reward_weights,
        )
        
        # --- episode metrics tracking ---
        self._episode_velocity_sum = torch.zeros(self.num_envs, device=self.device)
        self._episode_step_count = torch.zeros(self.num_envs, device=self.device)

        # debug vis
        self.set_debug_vis(self.cfg.debug_vis)

    # ------------------------------------------------------------------
    # Scene
    # ------------------------------------------------------------------

    def _setup_scene(self) -> None:
        self._robot = Articulation(self.cfg.robot)
        self.scene.articulations["robot"] = self._robot

        # Terrain (spawns gate meshes as part of the trimesh)
        self.cfg.terrain.num_envs = self.scene.cfg.num_envs
        self.cfg.terrain.env_spacing = self.scene.cfg.env_spacing
        print(f"[DEBUG] Creating terrain with {self.cfg.terrain.num_envs} envs, spacing {self.cfg.terrain.env_spacing}m")
        self._terrain = self.cfg.terrain.class_type(self.cfg.terrain)

        self.scene.clone_environments(copy_from_source=False)
        if self.device == "cpu":
            self.scene.filter_collisions(global_prim_paths=[self.cfg.terrain.prim_path])

        light_cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75))
        light_cfg.func("/World/Light", light_cfg)

    # ------------------------------------------------------------------
    # Gate tracking setup
    # ------------------------------------------------------------------

    def _setup_gate_tracking(self) -> None:
        """Extract gate data from the module-level registry and build tensors.

        After :class:`TerrainGenerator` runs, :func:`racing_gates_terrain` has
        deposited gate data keyed by ``(difficulty, seed)`` in the registry.

        Each cell in the terrain grid gets the *same* set of gates in its local
        frame (since RACING_GATES_SIMPLE_CFG uses a single sub-terrain with
        ``curriculum=False``).  We take the first entry from the registry,
        offset it by the per-cell ``terrain_origin``, and centre it exactly
        the same way the TerrainGenerator centres its meshes.
        """
        registry = get_gate_registry()
        if not registry:
            raise RuntimeError(
                "Gate registry is empty -- racing_gates_terrain was never called. "
                "Make sure the terrain generator uses MeshRacingGatesTerrainCfg."
            )

        # Take the first (and, with SIMPLE_CFG, only) entry
        gate_data = next(iter(registry.values()))
        print(f"[DEBUG] Found {len(gate_data.positions)} gates in registry")

        # Gate positions are in the sub-terrain's *raw* local frame.
        # The TerrainGenerator centres the mesh by shifting it by
        # (-size[0]/2, -size[1]/2) **before** placing it into the grid.
        # Then ``terrain_origins`` already accounts for that shift.
        # So we apply the same centring offset to the gate positions.
        terrain_cfg = self._terrain.cfg.terrain_generator
        centre_offset = np.array([-terrain_cfg.size[0] / 2.0, -terrain_cfg.size[1] / 2.0, 0.0])
        local_gate_pos = gate_data.positions + centre_offset  # [M, 3]

        base_gate_pos = torch.from_numpy(local_gate_pos).float().to(self.device)
        base_gate_ori = torch.from_numpy(gate_data.orientations).float().to(self.device)
        self._gate_size = gate_data.gate_size

        num_gates = base_gate_pos.shape[0]
        self._num_gates = num_gates

        # terrain_origins: [num_rows, num_cols, 3] -- already in world frame
        # TerrainImporter stores this directly (already as a Tensor on CPU)
        if self._terrain.terrain_origins is None:
            raise RuntimeError("Terrain origins are None - make sure use_terrain_origins=True in TerrainImporterCfg")
        
        terrain_origins = self._terrain.terrain_origins
        if isinstance(terrain_origins, np.ndarray):
            terrain_origins = torch.from_numpy(terrain_origins).float().to(self.device)
        else:
            terrain_origins = terrain_origins.to(self.device)
        num_rows, num_cols = terrain_origins.shape[:2]

        # Build per-env gate tensors by offsetting by the terrain origin
        # Env -> grid cell mapping (same logic as TerrainImporter)
        self._gate_positions = torch.zeros(self.num_envs, num_gates, 3, device=self.device)
        self._gate_orientations = base_gate_ori.unsqueeze(0).expand(self.num_envs, -1, -1).clone()

        for env_id in range(self.num_envs):
            row = (env_id // num_cols) % num_rows
            col = env_id % num_cols
            origin = terrain_origins[row, col]
            self._gate_positions[env_id] = base_gate_pos + origin

        # Tracking state (all vectorised)
        self._current_gate_idx = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)
        self._gates_passed = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)
        self._prev_drone_pos = torch.zeros(self.num_envs, 3, device=self.device)

        # Debug: Count gates across all environments
        total_gates = self.num_envs * num_gates
        gate_cfg = terrain_cfg.sub_terrains['racing_gates']
        print(f"[RacingQuadcopterEnv] Enhanced Racing Gate Configuration:")
        print(f"  Trajectory type: {gate_cfg.trajectory_type} (ENHANCED with complex 3D paths)")
        print(f"  Difficulty range: {terrain_cfg.difficulty_range[0]:.1f}-{terrain_cfg.difficulty_range[1]:.1f} (HIGH for maximum complexity)")
        print(f"  Gates per env: {num_gates} (target: ≥10 gates)")
        print(f"  Total environments: {self.num_envs}")
        print(f"  Total gates in simulation: {total_gates}")
        print(f"  Gate size: {self._gate_size:.3f}m")
        print(f"  Gate height range: {gate_cfg.gate_height_range[0]:.1f}-{gate_cfg.gate_height_range[1]:.1f}m (EXTENDED vertical range)")
        if hasattr(gate_cfg, 'gate_spacing_range') and gate_cfg.gate_spacing_range is not None:
            print(f"  Gate spacing: {gate_cfg.gate_spacing_range[0]:.1f}-{gate_cfg.gate_spacing_range[1]:.1f}m (tighter for more gates)")
        else:
            print(f"  Gate spacing: {gate_cfg.gate_spacing:.2f}m (fixed)")
        print(f"  Terrain size: {terrain_cfg.size[0]}x{terrain_cfg.size[1]}m (ENLARGED for longer trajectories)")
        print(f"  All gates perpendicular to ground with ENHANCED 3D trajectory complexity")

    # ------------------------------------------------------------------
    # Helpers: gather current-gate data for the whole batch
    # ------------------------------------------------------------------

    def _gather_current_gate(self):
        """Return pos [N,3], ori [N,4] of each env's current target gate."""
        idx = self._current_gate_idx  # [N]
        env_arange = torch.arange(self.num_envs, device=self.device)
        pos = self._gate_positions[env_arange, idx]
        ori = self._gate_orientations[env_arange, idx]
        return pos, ori

    def _gate_forward(self, gate_ori: torch.Tensor) -> torch.Tensor:
        """World-frame forward (y) direction of gates.  [N, 3]."""
        return quat_apply(gate_ori, self._y_local.expand(gate_ori.shape[0], -1))

    def _gate_right(self, gate_ori: torch.Tensor) -> torch.Tensor:
        """World-frame right (x) direction of gates.  [N, 3]."""
        return quat_apply(gate_ori, self._x_local.expand(gate_ori.shape[0], -1))

    def _gate_up(self, gate_ori: torch.Tensor) -> torch.Tensor:
        """World-frame up (z) direction of gates.  [N, 3]."""
        return quat_apply(gate_ori, self._z_local.expand(gate_ori.shape[0], -1))

    # ------------------------------------------------------------------
    # Step
    # ------------------------------------------------------------------

    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        self._previous_actions = self._actions.clone()
        self._actions = actions.clone().clamp(-1.0, 1.0)

        thrust_cmd = (self._actions[:, 0] + 1.0) * 0.5  # -> [0, 1]
        self._thrust[:, 0, 2] = self.cfg.thrust_to_weight * self._robot_weight * thrust_cmd
        self._moment[:, 0, :] = self.cfg.moment_scale * self._actions[:, 1:]

    def _apply_action(self) -> None:
        self._robot.set_external_force_and_torque(self._thrust, self._moment, body_ids=self._body_id)

    # ------------------------------------------------------------------
    # Observations (vectorised)
    # ------------------------------------------------------------------

    def _get_observations(self) -> dict:
        parts = [
            self._robot.data.root_lin_vel_b,       # [N, 3]
            self._robot.data.root_ang_vel_b,        # [N, 3]
            self._robot.data.projected_gravity_b,   # [N, 3]
        ]
        if self.cfg.observation.include_velocity_limit:
            parts.append(self._velocity_limit.unsqueeze(-1))  # [N, 1]

        parts.append(self._compute_gate_observations())  # [N, num_next_gates*6]

        if self.cfg.observation.history_length > 0:
            parts.append(self._lin_vel_history.reshape(self.num_envs, -1))
            parts.append(self._ang_vel_history.reshape(self.num_envs, -1))
            parts.append(self._action_history.reshape(self.num_envs, -1))

        return {"policy": torch.cat(parts, dim=-1)}

    def _compute_gate_observations(self) -> torch.Tensor:
        """Next-N-gate observations in body frame (fully vectorised).
        
        Returns position and forward direction for each of the next K gates,
        transformed into the drone's body frame.
        
        Output format per gate: [rel_pos_x, rel_pos_y, rel_pos_z, fwd_x, fwd_y, fwd_z]
        Total output shape: [N, K*6]
        """
        K = self.cfg.observation.num_next_gates
        M = self._num_gates
        N = self.num_envs

        drone_pos = self._robot.data.root_pos_w   # [N, 3]
        drone_quat = self._robot.data.root_quat_w  # [N, 4]

        # Pre-allocate output
        gate_obs = torch.zeros(N, K * 6, device=self.device)

        # Batch compute all gate indices at once
        env_arange = torch.arange(N, device=self.device)
        
        for i in range(K):
            gate_idx = (self._current_gate_idx + i) % M  # [N]
            
            # Gather gate data for all envs at once
            g_pos = self._gate_positions[env_arange, gate_idx]  # [N, 3]
            g_ori = self._gate_orientations[env_arange, gate_idx]  # [N, 4]

            # Relative position in world frame, then transform to body frame
            rel_w = g_pos - drone_pos  # [N, 3]
            rel_b = quat_apply_inverse(drone_quat, rel_w)  # [N, 3]

            # Normalize position for network input (scale by expected max distance)
            rel_b_normalized = rel_b / 10.0

            # Gate forward direction in world frame, then transform to body frame
            fwd_w = self._gate_forward(g_ori)  # [N, 3]
            fwd_b = quat_apply_inverse(drone_quat, fwd_w)  # [N, 3]

            # Write to output tensor
            s = i * 6
            gate_obs[:, s:s+3] = rel_b_normalized
            gate_obs[:, s+3:s+6] = fwd_b

        return gate_obs

    # ------------------------------------------------------------------
    # Rewards (vectorised via reward manager)
    # ------------------------------------------------------------------

    def _get_rewards(self) -> torch.Tensor:
        drone_pos = self._robot.data.root_pos_w
        drone_vel = self._robot.data.root_lin_vel_w
        gate_pos, gate_ori = self._gather_current_gate()
        
        # Check for gate passages
        gate_passed = self._check_gate_passages(drone_pos)
        
        # Check for crashes (ground collision or upside down)
        crashed = (
            (drone_pos[:, 2] < self.cfg.min_height) |
            (self._robot.data.projected_gravity_b[:, 2] > self.cfg.upside_down_threshold)
        )
        
        # Compute all rewards using manager
        total_reward, components = self._reward_manager.compute_rewards(
            # Robot state
            drone_pos=drone_pos,
            drone_vel=drone_vel,
            drone_angvel=self._robot.data.root_ang_vel_b,
            projected_gravity=self._robot.data.projected_gravity_b,
            actions=self._actions,
            prev_actions=self._previous_actions,
            # Gate state
            gate_pos=gate_pos,
            gate_ori=gate_ori,
            gate_size=self._gate_size,
            # Helper functions
            gate_forward_fn=self._gate_forward,
            gate_right_fn=self._gate_right,
            gate_up_fn=self._gate_up,
            # Previous state
            prev_drone_pos=self._prev_drone_pos,
            # Gate passage
            gate_passed=gate_passed,
            # Limits
            velocity_limit=self._velocity_limit,
            # Termination
            crashed=crashed,
        )
        
        # Track episode metrics
        speed = torch.norm(drone_vel, dim=1)
        self._episode_velocity_sum += speed
        self._episode_step_count += 1.0
        
        # Update previous position for next step
        self._prev_drone_pos = drone_pos.clone()
        
        return total_reward

    def _check_gate_passages(self, drone_pos: torch.Tensor) -> torch.Tensor:
        """Vectorised gate-plane crossing test for the full batch.

        Returns a ``[N]`` tensor of 1/0 for each env that passed its gate.
        """
        gate_pos, gate_ori = self._gather_current_gate()
        fwd = self._gate_forward(gate_ori)  # [N, 3]

        to_prev = self._prev_drone_pos - gate_pos
        to_curr = drone_pos - gate_pos

        dist_prev = (to_prev * fwd).sum(dim=1)   # signed distance to plane
        dist_curr = (to_curr * fwd).sum(dim=1)

        # crossed from behind (<=0) to front (>0)?
        crossed = (dist_prev <= 0) & (dist_curr > 0)

        # interpolate crossing point
        t = (-dist_prev / (dist_curr - dist_prev + 1e-8)).clamp(0.0, 1.0)
        crossing = self._prev_drone_pos + t.unsqueeze(-1) * (drone_pos - self._prev_drone_pos)
        to_cross = crossing - gate_pos

        right = self._gate_right(gate_ori)
        up = self._gate_up(gate_ori)
        lat = (to_cross * right).sum(dim=1).abs()
        vert = (to_cross * up).sum(dim=1).abs()

        half = self._gate_size / 2.0
        within = (lat < half) & (vert < half)

        passed = crossed & within  # [N] bool

        # advance gate index for envs that passed
        if passed.any():
            self._gates_passed[passed] += 1
            self._current_gate_idx[passed] = (self._current_gate_idx[passed] + 1) % self._num_gates

        return passed.float()

    # ------------------------------------------------------------------
    # Termination
    # ------------------------------------------------------------------

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        time_out = self.episode_length_buf >= self.max_episode_length - 1
        upside_down = self._robot.data.projected_gravity_b[:, 2] > self.cfg.upside_down_threshold
        crash = self._robot.data.root_pos_w[:, 2] < self.cfg.min_height
        runaway = torch.norm(self._robot.data.root_lin_vel_w, dim=1) > self.cfg.max_velocity
        died = upside_down | crash | runaway
        return died, time_out

    # ------------------------------------------------------------------
    # Reset
    # ------------------------------------------------------------------

    def _reset_idx(self, env_ids: torch.Tensor | None) -> None:
        if env_ids is None or len(env_ids) == self.num_envs:
            env_ids = torch.arange(self.num_envs, device=self.device)

        # --- logging ---
        if len(env_ids) > 0:
            # Get reward component averages from reward manager
            extras = self._reward_manager.reset_episode_sums(env_ids)
            
            # Add gate passage metric
            extras["Episode_Metrics/gates_passed"] = self._gates_passed[env_ids].float().mean().item()
            
            # Add mean velocity metric
            step_counts = self._episode_step_count[env_ids]
            valid_steps = step_counts > 0
            if valid_steps.any():
                mean_vel = (self._episode_velocity_sum[env_ids][valid_steps] / step_counts[valid_steps]).mean().item()
                extras["Episode_Metrics/mean_velocity"] = mean_vel
            else:
                extras["Episode_Metrics/mean_velocity"] = 0.0
            
            self.extras["log"] = extras
        
        # Reset episode metrics
        self._episode_velocity_sum[env_ids] = 0.0
        self._episode_step_count[env_ids] = 0.0

        super()._reset_idx(env_ids)

        # --- reset gate tracking ---
        self._current_gate_idx[env_ids] = 0
        self._gates_passed[env_ids] = 0

        # --- compute starting poses (2 m behind first gate, facing it) ---
        first_gate_pos = self._gate_positions[env_ids, 0]  # [B, 3]
        first_gate_ori = self._gate_orientations[env_ids, 0]  # [B, 4]
        fwd = quat_apply(first_gate_ori, self._y_local.expand(len(env_ids), -1))  # [B, 3]

        start_pos = first_gate_pos - 2.0 * fwd
        start_pos[:, 2] = first_gate_pos[:, 2]  # same height as gate

        root_poses = torch.zeros(len(env_ids), 7, device=self.device)
        root_poses[:, :3] = start_pos
        root_poses[:, 3:7] = first_gate_ori  # face the gate
        root_vel = torch.zeros(len(env_ids), 6, device=self.device)

        joint_pos = self._robot.data.default_joint_pos[env_ids]
        joint_vel = self._robot.data.default_joint_vel[env_ids]

        self._robot.write_root_pose_to_sim(root_poses, env_ids)
        self._robot.write_root_velocity_to_sim(root_vel, env_ids)
        self._robot.write_joint_state_to_sim(joint_pos, joint_vel, None, env_ids)

        self._prev_drone_pos[env_ids] = start_pos
        self._actions[env_ids] = 0.0
        self._previous_actions[env_ids] = 0.0

        if self.cfg.randomize_velocity_limit:
            lo, hi = self.cfg.velocity_limit_range
            self._velocity_limit[env_ids] = torch.rand(len(env_ids), device=self.device) * (hi - lo) + lo

    # ------------------------------------------------------------------
    # Debug visualisation
    # ------------------------------------------------------------------

    def _set_debug_vis_impl(self, debug_vis: bool) -> None:
        if debug_vis:
            if not hasattr(self, "gate_visualizer"):
                marker_cfg = CUBOID_MARKER_CFG.copy()
                marker_cfg.markers["cuboid"].size = (0.1, 0.1, 0.1)
                marker_cfg.markers["cuboid"].visual_material.diffuse_color = (0.0, 1.0, 0.0)
                marker_cfg.prim_path = "/Visuals/Command/current_gate"
                self.gate_visualizer = VisualizationMarkers(marker_cfg)
            self.gate_visualizer.set_visibility(True)
        else:
            if hasattr(self, "gate_visualizer"):
                self.gate_visualizer.set_visibility(False)

    def _debug_vis_callback(self, event) -> None:
        gate_pos, _ = self._gather_current_gate()
        self.gate_visualizer.visualize(gate_pos)
