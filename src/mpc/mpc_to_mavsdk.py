#!/usr/bin/env python3
"""
Main MPC control loop with thrust and body rate commands.
Connects to MAVSDK, executes MPC, sends offboard attitude rate commands.
"""
import asyncio
import csv
import numpy as np
import os
import time
from mavsdk import System
from mavsdk.offboard import AttitudeRate, PositionNedYaw

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


async def wait_for_connection(drone: System, timeout_s: float = 10.0):
    """Wait for MAVSDK connection."""
    print("Waiting connection...")
    t0 = time.time()
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected to vehicle")
            return True
        if time.time() - t0 > timeout_s:
            print("Connection timeout")
            return False
        await asyncio.sleep(0.1)


async def wait_until_armable(drone: System, timeout_s: float = 30.0):
    """Wait until vehicle health indicates armable state."""
    print("Waiting for vehicle to be armable...")
    t0 = time.time()
    async for health in drone.telemetry.health():
        print(f"Health: global={health.is_global_position_ok}, "
              f"local={health.is_local_position_ok}, "
              f"is_armable={health.is_armable}")
        if health.is_armable:
            return True
        if time.time() - t0 > timeout_s:
            print("Timed out waiting for armable")
            return False
        await asyncio.sleep(0.5)
    return False

def target_reached(current_pos, target_pos, tolerance):
    """
    Switch to next waypoint BEFORE reaching it.
    We shrink effective distance by a factor, causing smooth transitions.
    """
    dist = np.linalg.norm(current_pos - target_pos)

    # anticipative smoothing factor
    anticip = 0.65     # lower = earlier switching (0.5 = aggressive, 0.8 = mild)

    return dist <= (tolerance * anticip)


async def main():
    # ============================================
    # Initialize MPC
    # ============================================
    drone_model = DroneModel()
    opt = DroneOptimizer(d_model=drone_model.model,
                         d_constraint=drone_model.constraint,
                         t_horizon=1.5,
                         n_nodes=15,
                         enable_logging=True)

    # Reference state: hovering at (1, 1, -2) meters NED (2m altitude)
    # NED: North, East, Down (down is positive in PX4 convention)
    # ===============================
    # WAYPOINTS (add at top of main)
    # ===============================
    waypoints = [
        np.array([0.0, 0.0, -1.0]),
        np.array([1.0, 0.5, -1.2]),
        np.array([2.0, 1.0, -1.5]),
        np.array([3.5, 2.0, -2.0]),
        np.array([5.0, 3.0, -4.0]),
    ]

    current_wp_index = 0

    def yaw_to_quaternion(yaw):
        """Convert yaw (rad) to quaternion [w, x, y, z] with zero roll/pitch."""
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        return np.array([cy, 0.0, 0.0, sy])

    def compute_yaw_facing(current_pos, next_target):
        """Compute yaw so drone points toward next waypoint."""
        direction = next_target - current_pos
        yaw = np.arctan2(direction[1], direction[0])  # yaw around Z
        return yaw

    # Initialize first reference
    position = np.array([0.0, 0.0, 0.0])
    velocity = np.array([0.0, 0.0, 0.0])

    goal_pos = waypoints[current_wp_index]
    yaw_ref = compute_yaw_facing(position, goal_pos)
    q_ref = yaw_to_quaternion(yaw_ref)

    x_ref = np.zeros(13)
    x_ref[0:3] = goal_pos
    x_ref[6:10] = q_ref

    # ============================================
    # Connect to drone
    # ============================================
    drone = System()
    await drone.connect(system_address="udp://:14540")

    ok = await wait_for_connection(drone, timeout_s=15.0)
    if not ok:
        print("Connection failed")
        return
    ok = await wait_until_armable(drone)
    if not ok:
        print("Vehicle not armable")
        return

    print("Arming...")
    await drone.action.arm()

    # Set initial setpoint before starting offboard
    # Start with hover attitude rate command
    await drone.offboard.set_attitude_rate(AttitudeRate(0.0, 0.0, 0.0, 0.0))
    
    print("Starting offboard...")
    await drone.offboard.start()

    # ============================================
    # ============================================
    # Setup visualization and logging
    # ============================================
    goal_pos = x_ref[0:3].tolist()
    
    # Setup logs directory first
    logs_dir = "./logs"
    os.makedirs(logs_dir, exist_ok=True)
    
    # Video path in logs directory
    video_path = os.path.join(logs_dir, 'mpc_flight_thrust.mp4')
    print(f"Video will be saved to: {os.path.abspath(video_path)}")
    
    vis = MPCVisualizer(horizon=opt.N, goal=goal_pos, 
                       record_video=record_video,
                       video_filename=video_path,
                       video_fps=20)
    vis.start()
    
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

    rate_hz = 200.0
    period = 1.0 / rate_hz
    try:
        print("Control loop start (thrust + body rates)")
        iteration = 0
        
        while True:
            iteration += 1
            
            # ============================================
            # Get telemetry
            # ============================================
            async for pos in drone.telemetry.position_velocity_ned():
                p = pos.position
                v = pos.velocity
                position = np.array([p.north_m, p.east_m, p.down_m])
                velocity = np.array([v.north_m_s, v.east_m_s, v.down_m_s])
                break

            q = (1.0, 0.0, 0.0, 0.0)
            async for att in drone.telemetry.attitude_quaternion():
                q = (att.w, att.x, att.y, att.z)
                break

            ang_vel = np.array([0.0, 0.0, 0.0])
            try:
                async for odo in drone.telemetry.odometry():
                    ang_vel = np.array([odo.angular_velocity_body.roll_rad_s,
                                       odo.angular_velocity_body.pitch_rad_s,
                                       odo.angular_velocity_body.yaw_rad_s])
                    break
            except Exception:
                # Fallback: estimate from attitude changes
                ang_vel = np.array([0.0, 0.0, 0.0])

            x_current = build_state_from_telemetry(position, velocity, q, ang_vel)

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
                
                if iteration % 10 == 0:  # Print ogni 10 iterazioni
                    print(f"[MPC iter {iteration}] "
                          f"pos=({position[0]:.2f},{position[1]:.2f},{position[2]:.2f}) "
                          f"goal=({goal_pos[0]:.2f},{goal_pos[1]:.2f},{goal_pos[2]:.2f})")
                    print(f"  u=[T={thrust_N:.2f}N, "
                          f"wx={wx_cmd:.3f}, wy={wy_cmd:.3f}, wz={wz_cmd:.3f}]")
                    
            except Exception as e:
                print(f"[ERROR] Optimizer failed: {e}")
                # Emergency: send hover command
                try:
                    thrust_norm_hover = drone_mass * gravity / max_thrust
                    await drone.offboard.set_attitude_rate(
                        AttitudeRate(0.0, 0.0, 0.0, thrust_norm_hover)
                    )
                except Exception:
                    pass
                await asyncio.sleep(period)
                continue

            # ============================================
            # Send offboard command (thrust + body rates)
            # ============================================
            # MAVSDK AttitudeRate: roll_deg_s, pitch_deg_s, yaw_deg_s, thrust_value
            # thrust_value: [0,1] normalized thrust
            
            thrust_normalized = thrust_to_normalized(thrust_N, max_thrust)
            
            # Convert body rates from rad/s to deg/s
            roll_rate_deg = float(np.degrees(wx_cmd))
            pitch_rate_deg = float(np.degrees(wy_cmd))
            yaw_rate_deg = float(np.degrees(wz_cmd))
            
            # Safety clipping
            max_rate_deg = 200.0  # deg/s
            roll_rate_deg = np.clip(roll_rate_deg, -max_rate_deg, max_rate_deg)
            pitch_rate_deg = np.clip(pitch_rate_deg, -max_rate_deg, max_rate_deg)
            yaw_rate_deg = np.clip(yaw_rate_deg, -max_rate_deg, max_rate_deg)
            
            try:

                await drone.offboard.set_attitude_rate(AttitudeRate(roll_rate_deg, pitch_rate_deg, yaw_rate_deg, thrust_normalized)
                )
            except Exception as e:
                print(f"[ERROR] Offboard command failed: {e}")

            # ============================================
            # Logging
            # ============================================
            dist_to_goal = float(np.linalg.norm(position - np.array(goal_pos)))
            tstamp = time.time()
            qw, qx, qy, qz = q

            # Console (verbose ogni 10 iter)
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


            if target_reached(position, goal_pos, tolerance=0.3):
                print(f"Reached waypoint {current_wp_index}!")

                if current_wp_index < len(waypoints) - 1:
                    current_wp_index += 1
                    goal_pos = waypoints[current_wp_index]

                    # Update yaw orientation to face next goal
                    yaw_ref = compute_yaw_facing(position, goal_pos)
                    q_ref = yaw_to_quaternion(yaw_ref)

                    x_ref = np.zeros(13)
                    x_ref[0:3] = goal_pos
                    x_ref[6:10] = q_ref

                    print(f"Switching to waypoint {current_wp_index}: {goal_pos}")

                else:
                    print("🏁 Final waypoint reached — holding position.")

            # Wait for next control cycle
            await asyncio.sleep(period)

    except KeyboardInterrupt:
        print("\nKeyboard interrupt - landing")
        # Force save video before exit
        if record_video:
            print("Saving video before exit...")
    except Exception as e:
        print(f"\nUnexpected error: {e}")
        # Force save video before exit
        if record_video:
            print("Saving video before exit...")
    finally:
        print("Stopping offboard and disarming...")
        
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
        
        # Stop offboard and disarm
        try:
            await drone.offboard.stop()
        except Exception as e:
            print(f"Error stopping offboard: {e}")
        
        try:
            await drone.action.disarm()
        except Exception as e:
            print(f"Error disarming: {e}")


if __name__ == "__main__":
    os.chdir(os.path.dirname(os.path.realpath(__file__)))
    asyncio.run(main())