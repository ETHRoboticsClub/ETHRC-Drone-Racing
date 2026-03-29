#!/usr/bin/env python3
"""MPCC integration smoke-test with physics simulation (pre-Gazebo).

Usage:
  python src/mpcc/test.py

Output:
  - 3D plot with
      * waypoints (red x)
      * reference path (cyan line)
      * actual simulated trajectory (blue line)
      * MPC predicted horizon trajectories (grey low-alpha lines)
"""

import os
import sys

# Ensure project src directory is on Python path for local import resolution
SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
PROJECT_SRC = os.path.abspath(os.path.join(SCRIPT_DIR, '..'))
if PROJECT_SRC not in sys.path:
    sys.path.insert(0, PROJECT_SRC)

# If mavsdk is missing, provide a lightweight stand-in to allow local tests.
if 'mavsdk' not in sys.modules:
    import types
    mavsdk = types.ModuleType('mavsdk')
    offboard = types.ModuleType('mavsdk.offboard')

    class AttitudeRate:
        def __init__(self, roll_rate, pitch_rate, yaw_rate, thrust):
            self.roll_rate = roll_rate
            self.pitch_rate = pitch_rate
            self.yaw_rate = yaw_rate
            self.thrust = thrust

    offboard.AttitudeRate = AttitudeRate
    mavsdk.offboard = offboard
    sys.modules['mavsdk'] = mavsdk
    sys.modules['mavsdk.offboard'] = offboard

import numpy as np
import matplotlib.pyplot as plt

from mpl_toolkits.mplot3d import Axes3D

from mpcc.drone_mpcc_controller import DroneMpccController
from mpcc.drone_model import DroneMpccModel

try:
    import casadi as ca
except ImportError:
    raise RuntimeError("casadi is required to run mpcc/test.py")


def build_dynamics_integrator(model: DroneMpccModel):
    # model.model.x and model.model.u are CasADi MX symbols
    f_fun = ca.Function('f_expl', [model.model.x, model.model.u], [model.model.f_expl_expr])

    def step(x: np.ndarray, u: np.ndarray, dt: float) -> np.ndarray:
        # RK4 integration on continuous dynamics
        k1 = np.asarray(f_fun(x, u)).ravel()
        k2 = np.asarray(f_fun(x + 0.5 * dt * k1, u)).ravel()
        k3 = np.asarray(f_fun(x + 0.5 * dt * k2, u)).ravel()
        k4 = np.asarray(f_fun(x + dt * k3, u)).ravel()
        x_next = x + dt * (k1 + 2 * k2 + 2 * k3 + k4) / 6.0

        # Normalize quaternion to avoid drift
        qn = np.linalg.norm(x_next[3:7])
        if qn > 1e-8:
            x_next[3:7] /= qn

        # Keep yaw/pitch/roll consistently inside [-1,1]
        x_next[3:7] = x_next[3:7] / np.linalg.norm(x_next[3:7])
        return x_next

    return step


def main():
    # Waypoints in NED frame (z negative for downwards altitude)
    waypoints = np.array([
        [0.0, 0.0, -1.0],
        [3.0, 0.0, -1.0],
        [3.0, 3.0, -1.0],
        [0.0, 3.0, -1.0],
        [0.0, 0.0, -1.0],
    ], dtype=np.float64)

    mpcc = DroneMpccController(waypoints, t_horizon=1.0, n_nodes=20)
    print("Initializing MPCC solver (may take a while on first run)")
    mpcc.initialize()

    model = DroneMpccModel()
    integrator = build_dynamics_integrator(model)

    # Initial physical state (13 dims): [p(3), v(3), q(4), w(3)]
    x_ext = np.zeros(13, dtype=np.float64)
    x_ext[0:3] = waypoints[0]
    x_ext[6] = 1.0  # quaternion identity

    # Augmented internal state for simulation (19 dims)
    x_aug_phys = np.zeros(19, dtype=np.float64)
    x_aug_phys[0:3] = x_ext[0:3]
    x_aug_phys[3:7] = x_ext[6:10]
    x_aug_phys[7:10] = x_ext[3:6]
    x_aug_phys[10:13] = x_ext[10:13]
    x_aug_phys[13:17] = model.f_hover.copy()
    x_aug_phys[17] = 0.0
    x_aug_phys[18] = 0.0

    sim_rate_hz = 50.0
    dt = 1.0 / sim_rate_hz
    max_steps = 300

    actual_positions = []
    predicted_trajectories = []
    path_points = []

    for step in range(max_steps):
        u_opt, x_pred, info = mpcc._run_step(x_ext)
        if info is not None and hasattr(info, 'status'):
            status = info.status if hasattr(info, 'status') else None
        else:
            status = None

        # Collect MPC predicted reference trajectory
        predicted_trajectories.append(x_pred[:, 0:3].copy())

        # Convert MPC per-rotor predicted thrust to command
        f_opt = np.clip(x_pred[1, 13:17], model.F_MIN, model.F_MAX)

        # For physics simulation, use the optimizer's exact first-step control
        u_phys = u_opt.copy()

        # Step physics using full 19-d state
        # Ensure current physical thrust is in the state
        x_aug_phys[13:17] = x_aug_phys[13:17]
        x_aug_phys = integrator(x_aug_phys, u_phys, dt)

        # Update external state from simulated physical state
        x_ext[0:3] = x_aug_phys[0:3]
        x_ext[3:6] = x_aug_phys[7:10]
        x_ext[6:10] = x_aug_phys[3:7]
        x_ext[10:13] = x_aug_phys[10:13]

        actual_positions.append(x_ext[0:3].copy())

        if (step % 25) == 0:
            print(f"step {step:03d} theta={mpcc.current_theta:.2f} lap={mpcc.lap_count}"
                  f" status={status}")

    actual_positions = np.array(actual_positions)

    # Compute reference trajectory from path
    theta_query = np.linspace(0.0, mpcc.path_length, 400)
    ref_positions = np.array([mpcc._path.position(t) for t in theta_query])

    # Plot
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(ref_positions[:, 0], ref_positions[:, 1], ref_positions[:, 2],
            color='cyan', linestyle='-', linewidth=2, label='Reference Path')
    ax.plot(actual_positions[:, 0], actual_positions[:, 1], actual_positions[:, 2],
            color='blue', linestyle='-', linewidth=2, label='Simulated Physics Trajectory')

    # Plot predicted trajectories (subsample every 5)
    for idx in range(0, len(predicted_trajectories), 5):
        pred = predicted_trajectories[idx]
        ax.plot(pred[:, 0], pred[:, 1], pred[:, 2],
                color='gray', alpha=0.25, linewidth=1)

    ax.scatter(waypoints[:, 0], waypoints[:, 1], waypoints[:, 2],
               color='red', marker='x', s=80, label='Waypoints')

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('MPCC test: waypoints, reference path, simulated physics and predicted horizons')
    ax.legend()
    ax.grid(True)
    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    main()
