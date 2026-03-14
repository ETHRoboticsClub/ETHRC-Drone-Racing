#!/usr/bin/env python3
"""
Launch a PX4 X500 quadrotor in Isaac Sim using PegasusSimulator.

This replaces `make px4_sitl gz_x500` — PX4 SITL connects to Isaac Sim
via MAVLink HIL (TCP:4560) instead of Gazebo.

Usage:
    ISAACSIM_PYTHON launch_px4_isaacsim.py [--px4-dir PATH] [--headless]

After launch, connect MAVSDK / ROS 2 nodes as usual (PX4 listens on UDP 14540).
"""

import argparse
import os
import sys

# ---------------------------------------------------------------------------
# 1. Parse arguments BEFORE starting SimulationApp (Isaac Sim hijacks argv)
# ---------------------------------------------------------------------------
parser = argparse.ArgumentParser(description="PX4 + Isaac Sim via Pegasus")
parser.add_argument(
    "--px4-dir",
    type=str,
    default=os.environ.get("PX4_DIR", os.path.expanduser("~/ros_ws/px4/PX4-Autopilot")),
    help="Path to PX4-Autopilot directory (default: $PX4_DIR or ~/ros_ws/px4/PX4-Autopilot)",
)
parser.add_argument(
    "--vehicle-id",
    type=int,
    default=0,
    help="PX4 instance ID (default: 0). Each vehicle needs a unique ID.",
)
parser.add_argument("--headless", action="store_true", help="Run without rendering")
args, unknown = parser.parse_known_args()

# Re-inject unknown args so Isaac Sim can parse them
sys.argv = [sys.argv[0]] + unknown

# ---------------------------------------------------------------------------
# 2. Start Isaac Sim
# ---------------------------------------------------------------------------
from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": args.headless})

# ---------------------------------------------------------------------------
# 3. Configure Pegasus + PX4
# ---------------------------------------------------------------------------
from pegasus.simulator.params import ROBOTS  # noqa: E402
from pegasus.simulator.logic.backends.px4_mavlink_backend import (  # noqa: E402
    PX4MavlinkBackend,
    PX4MavlinkBackendConfig,
)
from pegasus.simulator.logic.graphical_sensors.monocular_camera import MonocularCamera  # noqa: E402
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig  # noqa: E402
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface  # noqa: E402

# Initialize Pegasus world
pegasus = PegasusInterface()
pegasus.load_environment_usd(
    # Default empty environment — replace with a custom USD scene if desired
    usd_file=os.environ.get("ISAAC_SCENE_USD", ""),
)
world = pegasus.world

# X500 quadrotor configuration
mavlink_config = PX4MavlinkBackendConfig(
    vehicle_id=args.vehicle_id,
    px4_autolaunch=True,
    px4_dir=args.px4_dir,
    px4_vehicle_model="gazebo-classic_iris",  # PX4 model string — HIL interface is model-agnostic
    enable_lockstep=True,
    num_rotors=4,
    input_offset=[0.0, 0.0, 0.0, 0.0],
    input_scaling=[1000.0, 1000.0, 1000.0, 1000.0],
    zero_position_armed=[100.0, 100.0, 100.0, 100.0],
)

multirotor_config = MultirotorConfig(
    usd_file=ROBOTS["Iris"],  # Default Iris mesh — can be swapped for X500 USD
    backends=[PX4MavlinkBackend(mavlink_config)],
)

# Spawn the drone at origin, 0.1 m above ground
Multirotor(
    "/World/quadrotor",
    multirotor_config,
    position=[0.0, 0.0, 0.1],
)

# ---------------------------------------------------------------------------
# 4. Reset and run the simulation loop
# ---------------------------------------------------------------------------
world.reset()

print("\n" + "=" * 60)
print("  Isaac Sim + PX4 ready!")
print(f"  PX4 dir:     {args.px4_dir}")
print(f"  Vehicle ID:  {args.vehicle_id}")
print(f"  MAVLink TCP: localhost:{4560 + args.vehicle_id}")
print(f"  MAVSDK UDP:  udp://:14540")
print("=" * 60 + "\n")

while simulation_app.is_running():
    world.step(render=not args.headless)

simulation_app.close()
