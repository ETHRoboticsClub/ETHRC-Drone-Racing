import asyncio
import argparse

from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw


async def main(args):
    drone = System()
    # Use udpin syntax to avoid the warning, but udp:// still works.
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    print("Waiting for drone...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("✅ Connected to drone")
            break

    print("Waiting for global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("✅ Global & home position OK")
            break

    # Ensure starting from a disarmed state and safe to arm
    is_armed = None
    async for armed in drone.telemetry.armed():
        is_armed = armed
        break

    if is_armed:
        print("⚠️ Drone is already armed. Landing & disarming before new mission...")
        try:
            await drone.action.land()
            # Wait a bit for landing to actually happen
            await asyncio.sleep(5)
        except Exception as e:
            print(f"Land before mission failed: {e}")

        # Wait until disarmed
        async for armed in drone.telemetry.armed():
            if not armed:
                print("✅ Disarmed, safe to start new mission")
                break
            await asyncio.sleep(0.5)

    print("Arming...")
    try:
        await drone.action.arm()
    except Exception as e:
        print(f"❌ Arming failed: {e}")
        return

    # Initial hover at 1 m above home
    initial_alt = 1.0  # z up
    initial_down = -initial_alt

    print("Starting Offboard mode...")
    await drone.offboard.set_position_ned(
        PositionNedYaw(0.0, 0.0, initial_down, 0.0)
    )

    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"❌ Offboard start failed: {e._result.result}")
        print("Disarming")
        await drone.action.disarm()
        return

    # --- Cartesian commands as before ---
    x = args.x
    y = args.y
    z = args.z
    yaw = args.yaw
    hold = args.hold

    north = x
    east = y
    down = -z
    
    rate_hz = 20.0
    dt = 1.0 / rate_hz
    steps = int(hold * rate_hz)
    
    for _ in range(steps):
        print(f"Going to x={x}, y={y}, z={z}, yaw={yaw}")
        await drone.offboard.set_position_ned(
            PositionNedYaw(north, east, down, yaw)
        )
        await asyncio.sleep(dt)
        
    
    for _ in range(steps):
        print(f"Holding position x={x}, y={y}, z={z}, yaw={yaw}")
        await drone.offboard.set_position_ned(
            PositionNedYaw(north, east, down, yaw)
        )
        await asyncio.sleep(dt)

    print("Landing...")
    await drone.action.land()

    # Wait until fully disarmed before exiting 
    async for armed in drone.telemetry.armed():
        if not armed:
            print("✅ Landed & disarmed, exiting cleanly")
            break
        await asyncio.sleep(0.5)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--x", type=float, default=0.0)
    parser.add_argument("--y", type=float, default=0.0)
    parser.add_argument("--z", type=float, default=6.0)
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--hold", type=float, default=8.0)
    args = parser.parse_args()

    asyncio.run(main(args))
