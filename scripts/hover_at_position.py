#!/usr/bin/env python3
"""
Basic MAVSDK script to hover drone 1m above spawn position.
The drone spawns at world (1, 1, 0) and flies to world (1, 1, 1).
This is NED (0, 0, -1) - 1m up from spawn.
Yaw is set to -135 degrees to face the origin.
"""

import asyncio
from mavsdk import System
from mavsdk.offboard import PositionNedYaw, OffboardError


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

    # Now command the target position: 1m up (z=-1), facing origin (-135deg)
    print("-- Flying to 1m altitude and hovering, facing origin")
    await drone.offboard.set_position_ned(
        PositionNedYaw(0.0, 0.0, -1.0, -135.0)
    )

    print("-- Press Ctrl+C to stop")

    # Hover - MAVSDK automatically sends setpoints at 20Hz
    # We don't need to continuously resend them
    try:
        while True:
            await asyncio.sleep(1)  # Just keep the loop alive
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
