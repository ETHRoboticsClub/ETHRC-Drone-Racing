#!/usr/bin/env python3
"""
Standalone MPC simulation with per-rotor thrust inputs.
Uses internal dynamics (DroneModelRotor) instead of MAVSDK.
This is the primary test/validation tool for the per-rotor MPC.
"""
import asyncio
import csv
import numpy as np
import os
import time

from drone_model_rotor import DroneModelRotor
from drone_opt_rotor import DroneOptimizerRotor
from mpc_vis import MPCVisualizer

record = 1
record_video = False if record == 0 else True


def build_state_from_telemetry(position, velocity, attitude_quat, ang_vel):
    """Build state vector: p(3), v(3), q(4), w(3)."""
    q0, q1, q2, q3 = attitude_quat
    x = np.zeros(13)
    x[0:3] = position
    x[3:6] = velocity
    x[6:10] = [q0, q1, q2, q3]
    x[10:13] = ang_vel
    return x


def compute_thrust_vector_ned(u_rotors, max_single_rotor_thrust, quaternion):
    """
    Compute total thrust vector in NED frame from per-rotor commands.

    Args:
        u_rotors: normalized per-rotor thrusts [4]
        max_single_rotor_thrust: max thrust per rotor (N)
        quaternion: attitude quaternion [w, x, y, z]

    Returns:
        thrust_vector_ned: (3,) thrust vector in NED frame
    """
    total_thrust = np.sum(u_rotors) * max_single_rotor_thrust
    thrust_body = np.array([0.0, 0.0, total_thrust])

    qw, qx, qy, qz = quaternion
    R_bn = np.array([
        [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy)],
        [2*(qx*qy + qw*qz), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qw*qx)],
        [2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1 - 2*(qx**2 + qy**2)]
    ])

    return R_bn @ thrust_body


def simulate_dynamics_step(x_current, u_control, dt, drone_model):
    """
    Simulate one step of per-rotor drone dynamics.

    Args:
        x_current: current state [13]
        u_control: per-rotor normalized thrust [4], each in [0, 1]
        dt: time step
        drone_model: DroneModelRotor instance

    Returns:
        x_next: next state after dt
    """
    f_dynamics = drone_model.f
    x_dot = f_dynamics(x_current, u_control).full().flatten()

    # Euler integration
    x_next = x_current + dt * x_dot

    # Normalize quaternion
    q = x_next[6:10]
    q_norm = np.linalg.norm(q)
    if q_norm > 1e-12:
        x_next[6:10] = q / q_norm

    return x_next


async def main():
    # ============================================
    # Initialize MPC with per-rotor model
    # ============================================
    drone_model = DroneModelRotor()
    opt = DroneOptimizerRotor(
        d_model=drone_model,
        t_horizon=2.0,
        n_nodes=20,
        hover_normalized=drone_model.hover_normalized,
        enable_logging=True)

    # Reference state: hovering at (1, 1, -2) meters NED
    x_ref = np.array([
        1.0, 1.0, -2.0,
        0.0, 0.0, 0.0,
        1.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0
    ], dtype=float)

    # ============================================
    # Initialize simulation state
    # ============================================
    print("Initializing simulated drone (per-rotor mode)...")

    initial_pos = np.array([0.0, 0.0, -0.5])
    initial_vel = np.array([0.0, 0.0, 0.0])
    initial_quat = np.array([1.0, 0.0, 0.0, 0.0])
    initial_ang_vel = np.array([0.0, 0.0, 0.0])

    x_current = build_state_from_telemetry(initial_pos, initial_vel, initial_quat, initial_ang_vel)

    print("Simulated drone ready (per-rotor control)")

    # ============================================
    # Setup visualization and logging
    # ============================================
    goal_pos = x_ref[0:3].tolist()

    logs_dir = "./logs"
    os.makedirs(logs_dir, exist_ok=True)

    video_path = os.path.join(logs_dir, 'mpc_rotor_simulation.mp4')
    print(f"Video will be saved to: {os.path.abspath(video_path)}")

    vis = MPCVisualizer(horizon=opt.N, goal=goal_pos,
                       record_video=record_video,
                       video_filename=video_path,
                       video_fps=20)
    vis.start()

    telemetry_log_path = os.path.join(logs_dir, "telemetry_log_rotor.csv")
    telemetry_log_file = open(telemetry_log_path, "w", newline="")
    telemetry_logger = csv.writer(telemetry_log_file)
    telemetry_logger.writerow(["time_s",
                               "north_m", "east_m", "down_m",
                               "vn_ms", "ve_ms", "vd_ms",
                               "qw", "qx", "qy", "qz",
                               "wx_rads", "wy_rads", "wz_rads",
                               "u0", "u1", "u2", "u3",
                               "total_thrust_N",
                               "dist_to_goal_m"])
    telemetry_log_file.flush()

    hover_norm = drone_model.hover_normalized
    max_single = drone_model.maxSingleRotorThrust

    print(f"\n{'='*60}")
    print(f"Per-rotor MPC simulation:")
    print(f"  Mass: {drone_model.m:.2f} kg")
    print(f"  Hover per-rotor: {hover_norm:.4f} (normalized)")
    print(f"  Hover per-rotor: {hover_norm * max_single:.2f} N")
    print(f"  Max single rotor: {max_single:.2f} N")
    print(f"  Total max thrust: {4*max_single:.2f} N")
    print(f"{'='*60}\n")

    rate_hz = 20.0
    period = 1.0 / rate_hz

    try:
        print("Control loop start (per-rotor mode)")
        iteration = 0

        while True:
            iteration += 1

            position = x_current[0:3]
            velocity = x_current[3:6]
            q = tuple(x_current[6:10])
            ang_vel = x_current[10:13]

            # ============================================
            # Compute MPC control
            # ============================================
            try:
                u0, x_next, x_pred_all, u_pred_all = await asyncio.to_thread(
                    opt.compute_control_once, x_current, x_ref
                )

                u_rotors = np.clip(u0, 0.0, 1.0)

                if iteration % 10 == 0:
                    total_thrust_N = np.sum(u_rotors) * max_single
                    print(f"[MPC-Rotor iter {iteration}] "
                          f"pos=({position[0]:.2f},{position[1]:.2f},{position[2]:.2f}) "
                          f"goal=({goal_pos[0]:.2f},{goal_pos[1]:.2f},{goal_pos[2]:.2f})")
                    print(f"  u_rotors=[{u_rotors[0]:.4f}, {u_rotors[1]:.4f}, "
                          f"{u_rotors[2]:.4f}, {u_rotors[3]:.4f}] "
                          f"T_total={total_thrust_N:.2f}N")

            except Exception as e:
                print(f"[ERROR] Optimizer failed: {e}")
                u_rotors = np.full(4, hover_norm)
                u0 = u_rotors

            # ============================================
            # Simulate dynamics step
            # ============================================
            try:
                x_current = simulate_dynamics_step(x_current, u_rotors, period, drone_model)
            except Exception as e:
                print(f"[ERROR] Dynamics simulation failed: {e}")
                break

            # ============================================
            # Logging
            # ============================================
            total_thrust_N = float(np.sum(u_rotors) * max_single)
            dist_to_goal = float(np.linalg.norm(position - np.array(goal_pos)))
            tstamp = time.time()
            qw, qx, qy, qz = q

            if iteration % 10 == 0:
                print(f"[telemetry] t={tstamp:.2f} "
                      f"att=(w={qw:.3f},x={qx:.3f},y={qy:.3f},z={qz:.3f}) "
                      f"|v|={np.linalg.norm(velocity):.2f} m/s "
                      f"T={total_thrust_N:.2f}N "
                      f"dist={dist_to_goal:.2f}m\n")

            telemetry_logger.writerow([
                f"{tstamp:.6f}",
                f"{position[0]:.6f}", f"{position[1]:.6f}", f"{position[2]:.6f}",
                f"{velocity[0]:.6f}", f"{velocity[1]:.6f}", f"{velocity[2]:.6f}",
                f"{qw:.6f}", f"{qx:.6f}", f"{qy:.6f}", f"{qz:.6f}",
                f"{ang_vel[0]:.6f}", f"{ang_vel[1]:.6f}", f"{ang_vel[2]:.6f}",
                f"{u_rotors[0]:.6f}", f"{u_rotors[1]:.6f}",
                f"{u_rotors[2]:.6f}", f"{u_rotors[3]:.6f}",
                f"{total_thrust_N:.6f}",
                f"{dist_to_goal:.6f}"
            ])
            telemetry_log_file.flush()

            # ============================================
            # Update visualizer
            # ============================================
            try:
                thrust_vector_ned = compute_thrust_vector_ned(u_rotors, max_single, q)
                vis.update(position, x_pred_all, u_pred_all,
                          current_velocity=velocity,
                          current_thrust_vector=thrust_vector_ned)
            except Exception as e_vis:
                print(f"Visualizer error: {e_vis}")

            await asyncio.sleep(period)

    except KeyboardInterrupt:
        print("\nKeyboard interrupt - stopping simulation")
    except Exception as e:
        print(f"\nUnexpected error: {e}")
    finally:
        print("Stopping simulation and cleaning up...")
        try:
            telemetry_log_file.close()
        except Exception:
            pass
        try:
            vis.stop(timeout=10.0)
            print("Visualizer stopped")
        except Exception:
            pass


if __name__ == "__main__":
    os.chdir(os.path.dirname(os.path.realpath(__file__)))
    asyncio.run(main())
