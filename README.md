# ETHRC-Drone-Racing

Dockerized environment for drone racing with:

- **ROS 2 Humble**
- **Gazebo Sim (gz)** + **ros_gz** bridge  
- **PX4 SITL**
- **MAVROS & MAVSDK**
- A ROS 2 workspace at `/ws`

> **Host OS:** Ubuntu/Linux recommended  
> **Requires:** Docker Engine + Compose v2

---

## Quick Start

### Step 0 — One-time: Set up SSH for GitHub (if you’ve never done this)

### Step 1 — Clone (with PX4 submodule)
```bash
git clone --recurse-submodules git@github.com:ETHRoboticsClub/ETHRC-Drone-Racing.git
cd ETHRC-Drone-Racing
```

### Step 2 — Build the Docker image
```bash
docker compose build
```

### Step 3 — Start services (enable GUI, start container, open shell)
```bash
# Allow container GUI apps to open windows on your desktop (X11)
xhost +local:root

# Start the container (detached) and open a shell inside
docker compose up -d
docker exec -it erc-aeronautics bash
```

### Step 4 — Run the simulation in the container
```bash
# Default x500 in the default world
cd /ws/px4/PX4-Autopilot
make px4_sitl gz_x500
```
