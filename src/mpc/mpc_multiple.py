#!/usr/bin/env python3
"""
Main MPC control loop with thrust and body rate commands.
Sequential waypoint navigation with multiple setpoints.
"""
import asyncio
import csv
import numpy as np
import os
import time
from mavsdk import System
from mavsdk.offboard import AttitudeRate

from drone_model import DroneModel
from drone_opt import DroneOptimizer
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


def thrust_to_normalized(thrust_N, max_thrust_N):
    """Convert thrust in Newtons to normalized [0,1] for MAVSDK."""
    return float(np.clip(thrust_N / max_thrust_N, 0.0, 1.0))


def compute_thrust_vector_ned(thrust_magnitude, quaternion):
    """Compute thrust vector in NED frame from thrust magnitude and drone orientation."""
    thrust_body = np.array([0.0, 0.0, thrust_magnitude])
    qw, qx, qy, qz = quaternion
    
    R_bn = np.array([
        [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy)],
        [2*(qx*qy + qw*qz), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qw*qx)],
        [2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1 - 2*(qx**2 + qy**2)]
    ])
    
    return R_bn @ thrust_body


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
        if health.is_armable:
            return True
        if time.time() - t0 > timeout_s:
            print("Timed out waiting for armable")
            return False
        await asyncio.sleep(0.5)
    return False


def create_reference_state(position_ned):
    """Create a reference state vector for hovering at a given position."""
    x_ref = np.array([
        position_ned[0], position_ned[1], position_ned[2],  # position NED
        0.0, 0.0, 0.0,                                       # velocity (hover)
        1.0, 0.0, 0.0, 0.0,                                  # quaternion (identity)
        0.0, 0.0, 0.0                                        # angular rates (zero)
    ], dtype=float)
    return x_ref


async def main():
    # ============================================
    # Waypoints configuration
    # ============================================
    waypoints = [
        np.array([2.0, 1.0, -2.0]),   # WP1: 2m alt
        np.array([4.0, 3.0, -3.0]),   # WP2: 3m alt
        np.array([6.0, 5.0, -4.0]),   # WP3: 4m alt
        np.array([4.0, 7.0, -3.5]),   # WP4: 3.5m alt
        np.array([2.0, 5.0, -2.5]),   # WP5: 2.5m alt
    ]
    
    waypoint_threshold = 0.3  # meters
    waypoint_dwell_time = 3.0  # seconds
    
    current_waypoint_idx = 0
    waypoint_reached_time = None
    
    print(f"\nMission: {len(waypoints)} waypoints, threshold={waypoint_threshold}m, dwell={waypoint_dwell_time}s\n")
    
    # ============================================
    # Initialize MPC
    # ============================================
    drone_model = DroneModel()
    opt = DroneOptimizer(d_model=drone_model.model,
                         d_constraint=drone_model.constraint,
                         t_horizon=1.5,
                         n_nodes=15,
                         enable_logging=True)

    x_ref = create_reference_state(waypoints[current_waypoint_idx])

    # ============================================
    # Connect to drone
    # ============================================
    drone = System()
    await drone.connect(system_address="udp://:14540")

    if not await wait_for_connection(drone, timeout_s=15.0):
        return
    if not await wait_until_armable(drone):
        return

    print("Arming...")
    await drone.action.arm()

    await drone.offboard.set_attitude_rate(AttitudeRate(0.0, 0.0, 0.0, 0.0))
    
    print("Starting offboard...")
    await drone.offboard.start()

    # ============================================
    # Setup logging and visualization
    # ============================================
    logs_dir = "./logs"
    os.makedirs(logs_dir, exist_ok=True)
    
    video_path = os.path.join(logs_dir, 'mpc_flight_waypoints.mp4')
    
    vis = MPCVisualizer(horizon=opt.N, 
                       goal=waypoints[current_waypoint_idx].tolist(), 
                       record_video=record_video,
                       video_filename=video_path,
                       video_fps=20,
                       waypoints=waypoints)
    vis.start()
    
    telemetry_log = open(os.path.join(logs_dir, "telemetry_log.csv"), "w", newline="")
    logger = csv.writer(telemetry_log)
    logger.writerow(["time_s", "north_m", "east_m", "down_m", 
                    "vn_ms", "ve_ms", "vd_ms",
                    "thrust_N", "wx_cmd", "wy_cmd", "wz_cmd",
                    "dist_to_goal_m", "current_waypoint"])
    telemetry_log.flush()

    # Physical parameters
    drone_mass = float(drone_model.m)
    gravity = float(drone_model.g)
    max_thrust = float(drone_model.constraint.thrust_max)
    
    rate_hz = 20.0
    period = 1.0 / rate_hz

    try:
        print(f"Control loop start\n")
        iteration = 0
        
        while True:
            iteration += 1
            
            # ============================================
            # Get telemetry
            # ============================================
            position = np.array([0.0, 0.0, 0.0])
            velocity = np.array([0.0, 0.0, 0.0])
            
            async for pos in drone.telemetry.position_velocity_ned():
                position = np.array([pos.position.north_m, pos.position.east_m, pos.position.down_m])
                velocity = np.array([pos.velocity.north_m_s, pos.velocity.east_m_s, pos.velocity.down_m_s])
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
            except:
                pass

            x_current = build_state_from_telemetry(position, velocity, q, ang_vel)

            # ============================================
            # Waypoint logic
            # ============================================
            current_goal = waypoints[current_waypoint_idx]
            dist_to_goal = float(np.linalg.norm(position - current_goal))
            
            # Check if waypoint reached
            if dist_to_goal < waypoint_threshold:
                if waypoint_reached_time is None:
                    waypoint_reached_time = time.time()
                    print(f"✓ WP{current_waypoint_idx + 1} reached (dist={dist_to_goal:.2f}m)")
                
                # Check if dwell time elapsed
                if (time.time() - waypoint_reached_time) >= waypoint_dwell_time:
                    current_waypoint_idx += 1
                    
                    # Mission complete
                    if current_waypoint_idx >= len(waypoints):
                        print(f"\nMission complete!\n")
                        await asyncio.sleep(2.0)
                        break
                    
                    # Switch to next waypoint
                    x_ref = create_reference_state(waypoints[current_waypoint_idx])
                    vis.goal = waypoints[current_waypoint_idx].tolist()
                    waypoint_reached_time = None
                    print(f"→ Switching to WP{current_waypoint_idx + 1}")
            else:
                waypoint_reached_time = None

            # ============================================
            # Compute MPC control
            # ============================================
            try:
                u0, x_next, x_pred_all, u_pred_all = await asyncio.to_thread(
                    opt.compute_control_once, x_current, x_ref
                )
                
                thrust_N = float(u0[0])
                wx_cmd = float(u0[1])
                wy_cmd = float(u0[2])
                wz_cmd = float(u0[3])
                
            except Exception as e:
                print(f"MPC error: {e}")
                thrust_N = drone_mass * gravity
                wx_cmd = wy_cmd = wz_cmd = 0.0

            # ============================================
            # Send command
            # ============================================
            thrust_normalized = thrust_to_normalized(thrust_N, max_thrust)
            
            roll_rate_deg = np.clip(np.degrees(wx_cmd), -200.0, 200.0)
            pitch_rate_deg = np.clip(np.degrees(wy_cmd), -200.0, 200.0)
            yaw_rate_deg = np.clip(np.degrees(wz_cmd), -200.0, 200.0)
            
            await drone.offboard.set_attitude_rate(
                AttitudeRate(roll_rate_deg, pitch_rate_deg, yaw_rate_deg, thrust_normalized)
            )

            # ============================================
            # Logging and visualization
            # ============================================
            if iteration % 20 == 0:
                print(f"WP{current_waypoint_idx + 1}/{len(waypoints)} "
                      f"pos=({position[0]:.1f},{position[1]:.1f},{position[2]:.1f}) "
                      f"dist={dist_to_goal:.2f}m")

            logger.writerow([
                f"{time.time():.3f}",
                f"{position[0]:.3f}", f"{position[1]:.3f}", f"{position[2]:.3f}",
                f"{velocity[0]:.3f}", f"{velocity[1]:.3f}", f"{velocity[2]:.3f}",
                f"{thrust_N:.2f}", f"{wx_cmd:.3f}", f"{wy_cmd:.3f}", f"{wz_cmd:.3f}",
                f"{dist_to_goal:.3f}", f"{current_waypoint_idx + 1}"
            ])
            telemetry_log.flush()

            try:
                thrust_vector = compute_thrust_vector_ned(thrust_N, q)
                vis.update(position, x_pred_all, u_pred_all,
                          goal=x_ref[0:3],
                          current_velocity=velocity,
                          current_thrust_vector=thrust_vector,
                          current_waypoint_idx=current_waypoint_idx)
            except:
                pass

            await asyncio.sleep(period)

    except KeyboardInterrupt:
        print("\nKeyboard interrupt")
    finally:
        print("\nCleaning up...")
        
        try:
            telemetry_log.close()
        except:
            pass
        
        try:
            vis.stop(timeout=10.0)
            print("Video saved")
        except:
            pass
        
        try:
            await drone.offboard.stop()
            await asyncio.sleep(0.5)
        except:
            pass
        
        try:
            await drone.action.land()
            await asyncio.sleep(2.0)
        except:
            pass
        
        try:
            await drone.action.disarm()
        except:
            pass
        
        print("Done")


if __name__ == "__main__":
    os.chdir(os.path.dirname(os.path.realpath(__file__)))
    asyncio.run(main())