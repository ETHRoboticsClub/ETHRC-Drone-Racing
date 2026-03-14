# Isaac Sim + Pegasus — Setup Guide

## Architecture

Pegasus replaces Gazebo as the simulator, while **everything else remains unchanged**:

```
┌─────────────────────────────────────────────────────────────┐
│  ROS 2 Nodes (unchanged)                                    │
│                                                             │
│  drone_controller_node ──► mavsdk_client ──► PX4 SITL      │
│        (MPC)                  (UDP:14540)     (Autopilot)   │
└─────────────────────────────┬───────────────────────────────┘
                              │ MAVLink HIL (TCP:4560)
                              ▼
                 ┌────────────────────────┐
                 │  PegasusSimulator      │
                 │  (PX4MavlinkBackend)   │
                 │                        │
                 │  ┌──────────────────┐  │
                 │  │   Isaac Sim      │  │
                 │  │   (Physics +     │  │
                 │  │    Rendering)    │  │
                 │  └──────────────────┘  │
                 └────────────────────────┘
```

### Data Flow

| Direction | MAVLink Messages | Content |
|-----------|-----------------|---------|
| Isaac Sim → PX4 | `HIL_SENSOR` | Accelerometer, gyroscope, magnetometer, barometer |
| Isaac Sim → PX4 | `HIL_GPS` | Lat/lon/alt, NED velocity |
| Isaac Sim → PX4 | `HIL_STATE_QUATERNION` | Ground truth: quaternion [w,x,y,z], angular velocities |
| PX4 → Isaac Sim | `HIL_ACTUATOR_CONTROLS` | Actuator commands (4 rotors) |

PX4 does not know whether it is talking to Gazebo or Isaac Sim — the HIL protocol is identical.

## Requirements

| Component | Version | Notes |
|-----------|---------|-------|
| Ubuntu | 22.04 LTS | |
| NVIDIA Driver | >= 550 | Required for Isaac Sim |
| GPU | NVIDIA RTX | **Minimum 8 GB VRAM** (RTX 3060+, RTX 4060+, or datacenter GPU) |
| Docker | 24+ | With Docker Compose v2 |
| NVIDIA Container Toolkit | latest | For GPU passthrough to Docker |

> **Important**: GPUs with less than 8 GB VRAM (e.g. RTX 3050 Laptop 4 GB) are **not supported** by Isaac Sim.

## Setup (Docker)

### 1. Install NVIDIA Container Toolkit (one-time, on host)

```bash
# Add NVIDIA repo
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
    | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
    | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
    | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# Install
sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit

# Restart Docker
sudo systemctl restart docker
```

### 2. Login to NVIDIA NGC (one-time, on host)

The base image `nvcr.io/nvidia/isaac-sim:4.2.0` requires a free NGC account.

```bash
# Create account at https://ngc.nvidia.com/signin
# Generate API key at https://ngc.nvidia.com/setup/api-key
docker login nvcr.io
# Username: $oauthtoken
# Password: <your NGC API key>
```

### 3. Build the container

```bash
cd ~/ETHRC-Drone-Racing  # project root
docker compose -f docker-compose.isaacsim.yaml build
```

First build downloads ~15 GB (Isaac Sim base image) and takes a while.

### 4. Start the container

```bash
docker compose -f docker-compose.isaacsim.yaml up -d
```

### 5. Enter the container

```bash
docker exec -it erc-isaacsim bash
```

### 6. Build PX4 for SITL (one-time, inside container)

```bash
cd ~/ros_ws/px4/PX4-Autopilot
make px4_sitl_default none
```

### 7. Build ROS 2 packages (one-time, inside container)

```bash
cd ~/ros_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to drone_control my_mavsdk px4_msgs
source install/setup.bash
```

## Running the Simulation

Open 4 terminals into the container:
```bash
docker exec -it erc-isaacsim bash
```

### Terminal 1 — Isaac Sim + PX4

```bash
/isaac-sim/python.sh sim/isaac/launch_px4_isaacsim.py --px4-dir ~/ros_ws/px4/PX4-Autopilot
```

Options:
| Flag | Default | Description |
|------|---------|-------------|
| `--px4-dir PATH` | `$PX4_DIR` or `~/ros_ws/px4/PX4-Autopilot` | Path to PX4-Autopilot |
| `--vehicle-id N` | `0` | PX4 instance ID (increment for multi-drone) |
| `--headless` | off | No rendering (saves VRAM, useful for CI/training) |

Wait for the `Isaac Sim + PX4 ready!` message before proceeding.

> **Note**: You must use `/isaac-sim/python.sh` (not `python3`). This is Isaac Sim's Python wrapper that sets up the required environment variables and paths. System `python3` does not have the `isaacsim` module.

### Terminal 2 — MPC Controller

```bash
ros2 run drone_control drone_controller_node
```

### Terminal 3 — MAVSDK Client

```bash
ros2 run my_mavsdk mavsdk_client
```

### Terminal 4 — Send a goal

```bash
ros2 run drone_control send_pose_goal goal_only -x 0.0 -y 0.0 -z -2.0 -yaw 0.0
```

Commands in terminals 2-4 are **identical** to those used with Gazebo.

## File Structure

```
sim/isaac/
├── README.md                  # This guide
├── setup_pegasus.sh           # Native setup script (non-Docker)
└── launch_px4_isaacsim.py     # Launch script (SimulationApp + PX4MavlinkBackend)

docker/
└── Dockerfile.isaacsim        # Based on nvcr.io/nvidia/isaac-sim:4.2.0

docker-compose.isaacsim.yaml   # Compose with NVIDIA GPU passthrough
```

## Version Compatibility

Pegasus Simulator versioning is **1:1 with Isaac Sim**. Each release only works with its matching Isaac Sim version.

| Isaac Sim image | Pegasus tag | Notes |
|----------------|-------------|-------|
| `nvcr.io/nvidia/isaac-sim:4.2.0` | `v4.2.0` | Current setup |
| `nvcr.io/nvidia/isaac-sim:4.5.0` | `v4.5.0` | |
| `nvcr.io/nvidia/isaac-sim:5.1.0` | `v5.1.0` (main branch) | Different `.kit` naming, not backward compatible |

To change version, update both `ISAACSIM_BASE_IMAGE` in `docker-compose.isaacsim.yaml` and the git tag in `Dockerfile.isaacsim`.

## Troubleshooting

### `CUDA libs are present, but no suitable CUDA GPU was found`
- GPU not visible inside the container. Check: `docker exec erc-isaacsim nvidia-smi`
- If `nvidia-smi` works but Isaac Sim still fails, your GPU likely has insufficient VRAM (need >= 8 GB)
- Try headless mode: add `--headless` flag (uses less VRAM but still needs ~4 GB)

### `No module named 'isaacsim'`
- You are using system `python3` instead of Isaac Sim's Python
- Use `/isaac-sim/python.sh` to run scripts

### `Permission denied: '/isaac-sim/kit/cache'`
- Non-critical warning. Isaac Sim cache directory is read-only for non-root users
- Does not affect simulation functionality

### PX4 does not connect
- Verify PX4 SITL is built: `ls ~/ros_ws/px4/PX4-Autopilot/build/px4_sitl_default/bin/px4`
- Pegasus uses TCP port `4560 + vehicle_id`. Check: `ss -tlnp | grep 4560`

### MAVSDK does not connect
- PX4 listens on UDP 14540 as usual. Check: `ss -ulnp | grep 14540`
- `ROS_DOMAIN_ID` must be the same across all terminals (default: 1)

### Lockstep slows down simulation
- Lockstep is enabled by default (deterministic). To disable: set `enable_lockstep=False` in `launch_px4_isaacsim.py`

### Container crashes on start (`runheadless.native.sh: Permission denied`)
- The compose file must override the entrypoint. Verify `entrypoint: ["/bin/bash"]` is in `docker-compose.isaacsim.yaml`

### `unknown or invalid runtime name: nvidia`
- NVIDIA Container Toolkit not installed. Follow step 1 of the setup
- After installing, restart Docker: `sudo systemctl restart docker`

## Differences from Gazebo

| Aspect | Gazebo | Isaac Sim + Pegasus |
|--------|--------|---------------------|
| Launch command | `make px4_sitl gz_x500` | `/isaac-sim/python.sh launch_px4_isaacsim.py` |
| Container | `erc-aeronautics` | `erc-isaacsim` |
| Compose file | `docker-compose.yaml` | `docker-compose.isaacsim.yaml` |
| Sim-PX4 protocol | MAVLink HIL (built-in) | MAVLink HIL (via Pegasus) |
| Transport | UDP | TCP |
| Physics engine | DART/Bullet | PhysX 5 |
| Rendering | Ogre2 | RTX (ray tracing) |
| GPU | Optional | Required (NVIDIA, >= 8 GB VRAM) |
| ROS 2 bridge | ros_gz_bridge | ROS2Backend (Pegasus) |
| MPC/MAVSDK nodes | Unchanged | Unchanged |
| Base image size | ~3 GB | ~15 GB |
