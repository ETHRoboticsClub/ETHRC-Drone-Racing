#!/usr/bin/env python3
"""
Orbit script that flies drone in a circular path while keeping camera pointed at origin.

World coordinates:
- Drone spawns at world (1, 1, 0)
- Flies to world (2.5, 1.5, 1.5) - the orbit start position
- Orbits with radius 1m around center (1.5, 1.5, 1.5) at height z=1.5
- Camera always points to world origin (0, 0, 0)
- Orbit speed: 0.25 m/s, clockwise (viewed from above)

Coordinate conversion (drone spawns at world (1,1,0)):
- World (x, y, z) -> NED (north, east, down)
- NED_north = world_x - 1
- NED_east = world_y - 1
- NED_down = -world_z (NED down is negative of world up)
"""

import asyncio
import math
from mavsdk import System
from mavsdk.offboard import PositionNedYaw, OffboardError


def world_to_ned(world_x, world_y, world_z):
    """Convert world coordinates to NED coordinates (relative to spawn at world 1,1,0)"""
    ned_north = world_x - 1.0
    ned_east = world_y - 1.0
    ned_down = -world_z
    return ned_north, ned_east, ned_down


def calculate_yaw_to_origin(world_x, world_y):
    """
    Calculate yaw angle (in degrees) to point camera at world origin (0,0,0)
    from position (world_x, world_y).

    Returns yaw in degrees, where:
    - 0° = North (positive X in world)
    - 90° = East (positive Y in world)
    - 180° = South (negative X in world)
    - -90° = West (negative Y in world)
    """
    # Vector from current position to origin
    dx = 0.0 - world_x  # Origin is at (0, 0, 0)
    dy = 0.0 - world_y

    # Calculate angle in radians, then convert to degrees
    # atan2(dy, dx) gives angle from positive X axis
    yaw_rad = math.atan2(dy, dx)
    yaw_deg = math.degrees(yaw_rad)

    return yaw_deg


async def run():
    # Create drone instance and connect
    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    print("-- Connection established")
    await asyncio.sleep(1)

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected!")
            break

    # Wait for drone to have a global position estimate
    print("Waiting for global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("Global position estimate OK")
            break

    # Arm FIRST (before offboard mode)
    print("-- Arming")
    await drone.action.arm()

    # Set initial setpoint to CURRENT position (0,0,0,0)
    print("-- Setting initial setpoint")
    await drone.offboard.set_position_ned(
        PositionNedYaw(0.0, 0.0, 0.0, 0.0)
    )

    # Start offboard mode
    print("-- Starting offboard mode")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed: {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return

    # Fly to orbit start position: world (2.5, 1.5, 1.5)
    print("-- Flying to orbit start position: world (2.5, 1.5, 1.5)")
    start_world_x, start_world_y, start_world_z = 2.5, 1.5, 1.5
    start_n, start_e, start_d = world_to_ned(start_world_x, start_world_y, start_world_z)
    start_yaw = calculate_yaw_to_origin(start_world_x, start_world_y)

    print(f"   NED: ({start_n:.1f}, {start_e:.1f}, {start_d:.1f}), Yaw: {start_yaw:.1f}°")

    await drone.offboard.set_position_ned(
        PositionNedYaw(start_n, start_e, start_d, start_yaw)
    )

    # Wait for drone to reach start position
    await asyncio.sleep(8)
    print("-- Reached orbit start position")

    # Orbit parameters
    orbit_center_x, orbit_center_y, orbit_z = 1.5, 1.5, 1.5  # World coordinates
    radius = 1.0  # meters
    velocity = 0.25  # m/s
    circumference = 2 * math.pi * radius
    orbit_period = circumference / velocity  # Time for one complete orbit

    print(f"-- Starting orbit:")
    print(f"   Center: world ({orbit_center_x}, {orbit_center_y}, {orbit_z})")
    print(f"   Radius: {radius}m, Velocity: {velocity}m/s")
    print(f"   Period: {orbit_period:.1f}s per orbit (clockwise)")
    print(f"   Camera always pointing to world origin (0, 0, 0)")
    print("-- Press Ctrl+C to stop")

    # Orbit loop - update position at 10 Hz
    update_rate_hz = 10
    dt = 1.0 / update_rate_hz
    elapsed_time = 0.0

    try:
        while True:
            # Angular position (clockwise means negative angular velocity)
            # Start at angle=0 (due East from center, which is world position (5, 3, 4))
            angle = -2 * math.pi * (elapsed_time / orbit_period)  # Negative for clockwise

            # Calculate world position on orbit
            world_x = orbit_center_x + radius * math.cos(angle)
            world_y = orbit_center_y + radius * math.sin(angle)
            world_z = orbit_z

            # Convert to NED
            ned_n, ned_e, ned_d = world_to_ned(world_x, world_y, world_z)

            # Calculate yaw to point at origin
            yaw = calculate_yaw_to_origin(world_x, world_y)

            # Send position command
            await drone.offboard.set_position_ned(
                PositionNedYaw(ned_n, ned_e, ned_d, yaw)
            )

            # Update time and sleep
            elapsed_time += dt
            await asyncio.sleep(dt)

    except KeyboardInterrupt:
        print("\n-- Stopping offboard mode")
        try:
            await drone.offboard.stop()
        except OffboardError as error:
            print(f"Stopping offboard mode failed: {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()


if __name__ == "__main__":
    asyncio.run(run())
