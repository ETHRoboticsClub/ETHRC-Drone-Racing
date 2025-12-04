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

## Agents
Coding agents should read the files in aeronautics-utility/agents and follow the coding conventions.