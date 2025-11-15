#!/usr/bin/env python3
"""
Main MPC control loop with thrust and body rate commands.
Uses internal drone dynamics model instead of MAVSDK for simulation.
"""
import asyncio
import csv
import numpy as np
import os
import time

from drone_model import DroneModel
from drone_opt import DroneOptimizer
from mpc_vis import MPCVisualizer

record = 1
record_video = False if record == 0 else True


def build_state_from_telemetry(position, velocity, attitude_quat, ang_vel):
    """Build state vector: p(3), v(3), q(4), w(3)."""
    q0, q1, q2, q3 = attitude_quat
    x = np.zeros(13)
    x[0:3] = position      # NED position
    x[3:6] = velocity      # NED velocity
    x[6:10] = [q0, q1, q2, q3]  # quaternion (w,x,y,z)
    x[10:13] = ang_vel     # body rates
    return x


def thrust_to_normalized(thrust_N, max_thrust_N):
    """Convert thrust in Newtons to normalized [0,1] for MAVSDK."""
    return float(np.clip(thrust_N / max_thrust_N, 0.0, 1.0))


def compute_thrust_vector_ned(thrust_magnitude, quaternion):
    """
    Compute thrust vector in NED frame from thrust magnitude and drone orientation.
    
    Args:
        thrust_magnitude: thrust magnitude in Newtons
        quaternion: attitude quaternion [w, x, y, z]
        
    Returns:
        thrust_vector_ned: (3,) thrust vector in NED frame
    """
    # Thrust vector in body frame (pointing up = +z_body)
    thrust_body = np.array([0.0, 0.0, thrust_magnitude])
    
    # Rotation matrix from body to NED using quaternion
    qw, qx, qy, qz = quaternion
    
    # Rotation matrix body -> NED
    R_bn = np.array([
        [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy)],
        [2*(qx*qy + qw*qz), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qw*qx)],
        [2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1 - 2*(qx**2 + qy**2)]
    ])
    
    # Transform thrust to NED frame
    thrust_ned = R_bn @ thrust_body
    
    return thrust_ned


def simulate_dynamics_step(x_current, u_control, dt, drone_model):
    """
    Simulate one step of drone dynamics using the internal model.
    This replaces the MAVSDK telemetry updates.
    
    Args:
        x_current: current state [13x1]
        u_control: control input [4x1] - [thrust, wx_cmd, wy_cmd, wz_cmd]
        dt: time step
        drone_model: DroneModel instance
        
    Returns:
        x_next: next state after dt
    """
    # Use the dynamics function from the model
    f_dynamics = drone_model.f
    
    # Compute state derivative
    x_dot = f_dynamics(x_current, u_control).full().flatten()
    
    # Simple Euler integration
    x_next = x_current + dt * x_dot
    
    # Normalize quaternion to maintain unit norm
    q = x_next[6:10]
    q_norm = np.linalg.norm(q)
    if q_norm > 1e-12:
        x_next[6:10] = q / q_norm
    
    return x_next


async def main():
    # ============================================
    # Initialize MPC
    # ============================================
    drone_model = DroneModel()
    opt = DroneOptimizer(d_model=drone_model.model,
                         d_constraint=drone_model.constraint,
                         t_horizon=2.0,
                         n_nodes=20,
                         enable_logging=True)

    # Reference state: hovering at (1, 1, -2) meters NED (2m altitude)
    # NED: North, East, Down (down is positive in PX4 convention)
    x_ref = np.array([
        1.0, 1.0, -2.0,           # position NED
        0.0, 0.0, 0.0,            # velocity NED
        1.0, 0.0, 0.0, 0.0,       # quaternion (hover = identity)
        0.0, 0.0, 0.0             # angular rates (zero)
    ], dtype=float)

    # ============================================
    # Initialize simulation state (replaces drone connection)
    # ============================================
    print("Initializing simulated drone state...")
    
    # Start from origin with slight offset
    initial_pos = np.array([0.0, 0.0, -0.5])  # Start at 0.5m altitude
    initial_vel = np.array([0.0, 0.0, 0.0])
    initial_quat = np.array([1.0, 0.0, 0.0, 0.0])  # Identity quaternion (no rotation)
    initial_ang_vel = np.array([0.0, 0.0, 0.0])
    
    # Current simulated state (replaces MAVSDK telemetry)
    x_current = build_state_from_telemetry(initial_pos, initial_vel, initial_quat, initial_ang_vel)
    
    print("Simulated drone 'connected' and 'armed'")
    print("Starting simulated 'offboard' mode...")

    # ============================================
    # Setup visualization and logging
    # ============================================
    goal_pos = x_ref[0:3].tolist()
    
    # Setup logs directory
    logs_dir = "./logs"
    os.makedirs(logs_dir, exist_ok=True)
    
    # Video path in logs directory
    video_path = os.path.join(logs_dir, 'mpc_simulation.mp4')
    print(f"Video will be saved to: {os.path.abspath(video_path)}")
    
    vis = MPCVisualizer(horizon=opt.N, goal=goal_pos, 
                       record_video=record_video,
                       video_filename=video_path,
                       video_fps=20)
    vis.start()

    logs_dir = "./logs"
    os.makedirs(logs_dir, exist_ok=True)
    
    telemetry_log_path = os.path.join(logs_dir, "telemetry_log_thrust.csv")
    telemetry_log_file = open(telemetry_log_path, "w", newline="")
    telemetry_logger = csv.writer(telemetry_log_file)
    telemetry_logger.writerow(["time_s",
                               "north_m", "east_m", "down_m",
                               "vn_ms", "ve_ms", "vd_ms",
                               "qw", "qx", "qy", "qz",
                               "wx_rads", "wy_rads", "wz_rads",
                               "thrust_N", "thrust_norm",
                               "wx_cmd", "wy_cmd", "wz_cmd",
                               "dist_to_goal_m"])
    telemetry_log_file.flush()

    # Physical parameters
    drone_mass = float(drone_model.m)
    gravity = float(drone_model.g)
    max_thrust = float(drone_model.constraint.thrust_max)
    
    print(f"\n{'='*60}")
    print(f"Drone parameters:")
    print(f"  Mass: {drone_mass:.2f} kg")
    print(f"  Hover thrust: {drone_mass * gravity:.2f} N")
    print(f"  Max thrust: {max_thrust:.2f} N")
    print(f"{'='*60}\n")

    rate_hz = 20.0
    period = 1.0 / rate_hz

    try:
        print("Control loop start (thrust + body rates)")
        iteration = 0
        
        while True:
            iteration += 1
            
            # ============================================
            # Get telemetry (from simulation instead of MAVSDK)
            # ============================================
            position = x_current[0:3]
            velocity = x_current[3:6]
            q = tuple(x_current[6:10])  # (w, x, y, z)
            ang_vel = x_current[10:13]

            # ============================================
            # Compute MPC control
            # ============================================
            try:
                u0, x_next, x_pred_all, u_pred_all = await asyncio.to_thread(
                    opt.compute_control_once, x_current, x_ref
                )
                
                thrust_N = float(u0[0])
                wx_cmd = float(u0[1])  # roll rate
                wy_cmd = float(u0[2])  # pitch rate
                wz_cmd = float(u0[3])  # yaw rate
                
                if iteration % 10 == 0:  # Print every 10 iterations
                    print(f"[MPC iter {iteration}] "
                          f"pos=({position[0]:.2f},{position[1]:.2f},{position[2]:.2f}) "
                          f"goal=({goal_pos[0]:.2f},{goal_pos[1]:.2f},{goal_pos[2]:.2f})")
                    print(f"  u=[T={thrust_N:.2f}N, "
                          f"wx={wx_cmd:.3f}, wy={wy_cmd:.3f}, wz={wz_cmd:.3f}]")
                    
            except Exception as e:
                print(f"[ERROR] Optimizer failed: {e}")
                # Emergency: send hover command
                thrust_N = drone_mass * gravity
                wx_cmd = wy_cmd = wz_cmd = 0.0
                u0 = np.array([thrust_N, wx_cmd, wy_cmd, wz_cmd])

            # ============================================
            # Apply control (simulation instead of MAVSDK offboard)
            # ============================================
            # MAVSDK AttitudeRate: roll_deg_s, pitch_deg_s, yaw_deg_s, thrust_value
            # thrust_value: [0,1] normalized thrust
            
            thrust_normalized = thrust_to_normalized(thrust_N, max_thrust)
            
            # Convert body rates from rad/s to deg/s (for logging compatibility)
            roll_rate_deg = float(np.degrees(wx_cmd))
            pitch_rate_deg = float(np.degrees(wy_cmd))
            yaw_rate_deg = float(np.degrees(wz_cmd))
            
            # Safety clipping (even though we're simulating)
            max_rate_deg = 200.0  # deg/s
            roll_rate_deg = np.clip(roll_rate_deg, -max_rate_deg, max_rate_deg)
            pitch_rate_deg = np.clip(pitch_rate_deg, -max_rate_deg, max_rate_deg)
            yaw_rate_deg = np.clip(yaw_rate_deg, -max_rate_deg, max_rate_deg)
            
            # Convert back to rad/s for simulation
            wx_cmd_clipped = np.radians(roll_rate_deg)
            wy_cmd_clipped = np.radians(pitch_rate_deg)
            wz_cmd_clipped = np.radians(yaw_rate_deg)
            
            u_clipped = np.array([thrust_N, wx_cmd_clipped, wy_cmd_clipped, wz_cmd_clipped])
            
            # ============================================
            # Simulate dynamics step (replaces MAVSDK physics)
            # ============================================
            try:
                x_current = simulate_dynamics_step(x_current, u_clipped, period, drone_model)
            except Exception as e:
                print(f"[ERROR] Dynamics simulation failed: {e}")
                break

            # ============================================
            # Logging
            # ============================================
            dist_to_goal = float(np.linalg.norm(position - np.array(goal_pos)))
            tstamp = time.time()
            qw, qx, qy, qz = q

            # Console (verbose every 10 iterations)
            if iteration % 10 == 0:
                print(f"[telemetry] t={tstamp:.2f} "
                      f"att=(w={qw:.3f},x={qx:.3f},y={qy:.3f},z={qz:.3f}) "
                      f"|v|={np.linalg.norm(velocity):.2f} m/s "
                      f"T={thrust_N:.2f}N ({thrust_normalized:.2f}) "
                      f"dist={dist_to_goal:.2f}m\n")

            # CSV logging
            telemetry_logger.writerow([
                f"{tstamp:.6f}",
                f"{position[0]:.6f}", f"{position[1]:.6f}", f"{position[2]:.6f}",
                f"{velocity[0]:.6f}", f"{velocity[1]:.6f}", f"{velocity[2]:.6f}",
                f"{qw:.6f}", f"{qx:.6f}", f"{qy:.6f}", f"{qz:.6f}",
                f"{ang_vel[0]:.6f}", f"{ang_vel[1]:.6f}", f"{ang_vel[2]:.6f}",
                f"{thrust_N:.6f}", f"{thrust_normalized:.6f}",
                f"{wx_cmd:.6f}", f"{wy_cmd:.6f}", f"{wz_cmd:.6f}",
                f"{dist_to_goal:.6f}"
            ])
            telemetry_log_file.flush()

            # ============================================
            # Update visualizer
            # ============================================
            try:
                # Compute thrust vector in NED frame for visualization
                thrust_vector_ned = compute_thrust_vector_ned(thrust_N, q)
                
                # Update visualizer with velocity and thrust vectors
                vis.update(position, x_pred_all, u_pred_all, 
                          current_velocity=velocity, 
                          current_thrust_vector=thrust_vector_ned)
            except Exception as e_vis:
                print(f"Visualizer error: {e_vis}")

            # Wait for next control cycle
            await asyncio.sleep(period)

    except KeyboardInterrupt:
        print("\nKeyboard interrupt - stopping simulation")
        # Force save video before exit
        if record_video:
            print("Saving video before exit...")
    except Exception as e:
        print(f"\nUnexpected error: {e}")
        # Force save video before exit  
        if record_video:
            print("Saving video before exit...")
    finally:
        print("Stopping simulation and cleaning up...")
        
        # Close files
        try:
            telemetry_log_file.close()
        except Exception:
            pass
        
        # Stop visualizer (this will save the video)
        try:
            vis.stop(timeout=10.0)  # Longer timeout for video saving
            print("Visualizer stopped")
        except Exception:
            pass


if __name__ == "__main__":
    os.chdir(os.path.dirname(os.path.realpath(__file__)))
    asyncio.run(main())