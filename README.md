# ETHRC-Drone-Racing

This repo gives us a ready-to-use drone racing environment in Docker.
Why Docker? → So everyone works in the same environment, avoiding “it works on my machine” issues.

It bundles:

ROS 2 Humble → our robotics framework

Gazebo (gz) → 3D simulator for drones

PX4 SITL → autopilot software in simulation

MAVROS / MAVSDK → talk between PX4 and ROS

Workspace /ws → where we put our ROS 2 packages

> **Host OS:** Ubuntu/Linux recommended  
> **Requires:** Docker Engine + Compose v2

---

## Quick Start

Note: after step 3, the container will be running in the background. You can stop it with `docker compose down`.
You can access the documentation at [http://localhost:8000](http://localhost:8000).

### Step 0 — One-time: Set up SSH for GitHub (if you’ve never done this)
We need SSH so git clone works smoothly without typing passwords.
Follow GitHub’s guide: [Follow Github's guide here](https://docs.github.com/en/authentication/connecting-to-github-with-ssh)

### Step 1 — Clone (with Submodules)
Ensure you clone with all submodules (PX4, OpenVINS, etc.) to avoid build errors.
```bash
git clone --recurse-submodules git@github.com:ETHRoboticsClub/ETHRC-Drone-Racing.git
cd ETHRC-Drone-Racing
# If you already cloned without submodules:
git submodule update --init --recursive
```

### Step 2 — Build the Docker image
```bash
docker compose build
```

### Step 2b — Prepare host ROS 2 build/log directories
Run once (or after permission issues) on the host to create `.colcon/ros2_ws/{build,install,log}` with your user owning them:
```bash
sudo ./scripts/prepare_colcon_volumes.sh
```

### Step 3 — Start services (enable GUI, start container, open shell)
```bash
# Allow container GUI apps to open windows on your desktop (X11)
xhost +local:root
# Start the container (detached) and open a shell inside
docker compose up -d
docker exec -it erc-aeronautics bash
```

### Step 4 - Build the whole pipeline 
```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

---

## Visualization & Monitoring

To "see" what the system is doing, you can use the following tools:

### 1. Gazebo (The Simulator)
Launch the simulation to see the drone in the 3D world:
```bash
source install/setup.bash
cd /ws/px4/PX4-Autopilot
make px4_sitl gz_x500_cam 
```

### 2. RViz (The Monitoring Tool)
Launch the perception stack with RViz visualization:
```bash
source install/setup.bash
# Launch ArUco estimation with RViz enabled
ros2 launch aruco_pose_estimation aruco_pose_estimation.launch.py rviz:=true
```

### 3. Full Perception Stack
Orchestrate VIO, ArUco, and EKF nodes:
```bash
source install/setup.bash
ros2 launch launch/perception.launch.py
```

---

## Recent Fixes (Feb 2026)
- **Submode Initialization**: Recursive submodules (`open_vins`, `vicon-receiver`) are now mandatory for a full build.
- **MPC API Update**: Fixed `acados_template` compatibility issues in `drone_opt.py` to support newer versions of the library.
- **MAVSDK Migration**: Updated drone controller node to use the standard `mavsdk.offboard` types.

## Agents
Coding agents should read the files in `aeronautics-utility/agents` and follow the coding conventions. Additionally, the AI should digest our documentation found under `docs/docs`, especially the development guidelines.

## Adding Custom Worlds

Place worlds here:
```bash
px4/PX4-Autopilot/Tools/simulation/gz/worlds/
```

Set which world is loaded in:
```bash
px4/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/4022_gz_x500_cam
PX4_GZ_WORLD=<world_name>
```

## Command the drone to fly
Use Mavsdk API (offboard or action library) or Qgroundcontrol 
