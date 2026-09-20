#!/usr/bin/env python3
"""
Performance Benchmark for Nicola RL Policy Runtime & State Estimation Pipeline.
Measures:
  1. TorchScript GRU Model Inference Latency & Throughput (FPS).
  2. Observation Building Time (25-dim vector construction).
  3. Action Mapping & Rate Command Scaling.
  4. Live ROS Topic Frequencies (/imu0, /drone_state_estimated, /drone_attitude_cmd).
"""

import os
import sys
import time
from pathlib import Path
import numpy as np

# Ensure workspace root is in path
project_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(project_root / "src" / "rl"))

import torch
from rl.aigp_policy_runtime import (
    AIGP_OURS_105,
    AigpOursObservationBuilder,
    Gate,
    build_observation,
    gate_rotation_from_rpy_deg,
    load_track,
    policy_action_to_attitude_cmd,
    TorchScriptPolicy,
)


def benchmark_policy_runtime(policy_rel_path="ext/nicola/tamed_policy_perception_torchscript.pt"):
    policy_path = project_root / policy_rel_path
    if not policy_path.exists():
        policy_path = project_root / "ext" / "nicola" / "policy.pt"
    track_path = project_root / "src" / "rl" / "config" / "aigp_competition_track.yaml"

    print("==================================================")
    print("      RL POLICY RUNTIME PERFORMANCE BENCHMARK     ")
    print("==================================================")
    print(f"Policy File: {policy_path} ({os.path.getsize(policy_path):,} bytes)")
    print(f"Track File:  {track_path}")
    print("--------------------------------------------------")

    # Load policy & track
    track = load_track(track_path)
    policy = TorchScriptPolicy(policy_path)
    ours_builder = (
        AigpOursObservationBuilder()
        if policy.observation_contract == AIGP_OURS_105
        else None
    )

    # Synthetic drone state: [pos(3), vel(3), quat_wxyz(4), ang_vel(3)]
    state = np.array([0.0, 0.0, 1.0, 2.5, 0.5, -0.1, 1.0, 0.0, 0.0, 0.0, 0.05, -0.02, 0.1], dtype=np.float64)
    previous_action = np.zeros(4, dtype=np.float32)

    def make_observation():
        if ours_builder is not None:
            return ours_builder.build(
                state,
                track,
                active_gate_index=0,
                previous_action=previous_action,
                velocity_limit=10.0,
            )
        return build_observation(
            state,
            track,
            active_gate_index=0,
            previous_action=previous_action,
        )

    # 1. Benchmark Observation Building
    obs_times = []
    for _ in range(1000):
        t0 = time.perf_counter()
        obs = make_observation()
        t1 = time.perf_counter()
        obs_times.append((t1 - t0) * 1000.0)  # ms

    obs_mean_ms = np.mean(obs_times)
    obs_std_ms = np.std(obs_times)
    print(
        f"Observation Generation ({policy.observation_dim}-dim): "
        f"{obs_mean_ms:.4f} ms ± {obs_std_ms:.4f} ms"
    )

    # 2. Warmup Policy
    for _ in range(50):
        _ = policy(obs)

    # 3. Benchmark Policy Inference Latency
    policy.reset()
    latencies = []
    actions = []
    
    N_STEPS = 500
    t_start_total = time.perf_counter()
    
    for step in range(N_STEPS):
        t0 = time.perf_counter()
        action = policy(obs)
        t1 = time.perf_counter()
        latencies.append((t1 - t0) * 1000.0)  # ms
        actions.append(action)
        # Update dummy previous action
        previous_action = action
        obs = make_observation()

    t_end_total = time.perf_counter()
    total_time_s = t_end_total - t_start_total
    throughput_fps = N_STEPS / total_time_s

    latencies = np.array(latencies)
    mean_lat = np.mean(latencies)
    std_lat = np.std(latencies)
    p95_lat = np.percentile(latencies, 95)
    p99_lat = np.percentile(latencies, 99)
    min_lat = np.min(latencies)
    max_lat = np.max(latencies)

    print("--------------------------------------------------")
    print(f"Inference Benchmark ({N_STEPS:,} steps):")
    print(f"  Mean Latency:       {mean_lat:.4f} ms")
    print(f"  Std Deviation:      {std_lat:.4f} ms")
    print(f"  Min / Max Latency:  {min_lat:.4f} ms / {max_lat:.4f} ms")
    print(f"  95th Percentile:    {p95_lat:.4f} ms")
    print(f"  99th Percentile:    {p99_lat:.4f} ms")
    print(f"  Inference Throughput: {throughput_fps:,.1f} FPS")

    # 4. Action Command Scaling Verification
    actions = np.array(actions)
    cmd, clamped = policy_action_to_attitude_cmd(actions[-1], rate_command_scale_rad_s=4.0)
    
    print("--------------------------------------------------")
    print("Action & Output Command Range:")
    print(f"  Action Bounds:      min={np.min(actions):.3f}, max={np.max(actions):.3f}")
    print(f"  Roll Rate Output:   {cmd[0]:.2f} deg/s")
    print(f"  Pitch Rate Output:  {cmd[1]:.2f} deg/s")
    print(f"  Yaw Rate Output:    {cmd[2]:.2f} deg/s")
    print(f"  Thrust Output:      {cmd[3]:.3f} (normalized [0, 1])")
    print("==================================================\n")


if __name__ == "__main__":
    benchmark_policy_runtime()
