# Getting Started

**Page Owner:** [Leads of the Aeronautics Division](https://github.com/orgs/ETHRoboticsClub/teams/aeronautics/)

---

Welcome to the ETHRC Drone Racing project! This repository provides a high-performance development environment for autonomous drone racing, featuring advanced perception, control, and simulation tools.

## 📜 Project History & Evolution

The project has evolved through several stages to achieve competitive racing performance:
1.  **Initial Phase**: Focused on PX4 Autopilot with MAVSDK/MAVROS for offboard control.
2.  **Performance Optimization**: Identified latencies in the PX4 offboard pipeline that limited extreme agility.
3.  **Current State**: We use **Betaflight** as our primary flight controller firmware for all current hardware deployments. Betaflight's racing-optimized PID loops and ultra-low-latency protocols provide the agility required for high-speed autonomous flight.

> [!IMPORTANT]
> **FC Firmware**: We use **Betaflight** for now as our core flight controller firmware. Simulation still supports PX4 SITL for initial testing, but the real-world focus is on the Betaflight/MSP stack.

---

## 🚀 Key Features

*   **Dockerized Environment**: Consistent development across host systems.
*   **High-Speed Perception**: 90Hz+ camera pipelines and optimized VIO.
*   **Advanced Control**: MPC and RL-based controllers for time-optimal racing.
*   **Betaflight Integration**: Low-latency control via the MSP protocol and high-rate Blackbox telemetry.

---

## 🛠️ Environment Setup

We use Docker to containerize our development environment, ensuring consistency and simplifying the setup of ROS 2 Humble and its dependencies.

### Step 0: Prerequisites
*   **Host OS**: Ubuntu 22.04+ (Highly recommended for GUI compatibility).
*   **Docker Engine + Compose v2**.
*   **SSH Setup**: [Follow GitHub's SSH guide](https://docs.github.com/en/authentication/connecting-to-github-with-ssh) to enable seamless git operations.

### Step 1: Clone the Repository
Ensure you clone with submodules to include all dependencies (VIO, Vicon, etc.).
```bash
git clone --recurse-submodules git@github.com:ETHRoboticsClub/ETHRC-Drone-Racing.git
cd ETHRC-Drone-Racing
git submodule update --init --recursive
```

### Step 2: Build the Environment
We use multi-stage builds to provide specialized environments for different teams.

```bash
# To build the FULL development environment (Recommended)
docker compose build dev

# Or build for a specific team:
# docker compose build perception
# docker compose build mpc
# docker compose build rl
```

### Step 3: Start Services
Allow GUI applications (like RViz and Gazebo) and start the container.
```bash
xhost +local:root
docker compose up dev -d
```

### Step 4: Access and Build Workspace
Attach to the container and run the initial build.
```bash
docker compose exec dev bash

# Inside the container:
./scripts/prepare_colcon_volumes.sh # Ensure volumes are ready
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

---

## 📂 Navigating the Documentation

Once you are set up, explore the following sections for more details:

*   [**Perception Stack**](perception/index.md): VIO, Gate Detection, and Sensor Fusion.
*   [**Controls & Planning**](controls/index.md): MPC, RL, and how to [**Fly the Drone**](controls/flying_the_drone.md).
*   [**Drone Setup**](drone-setup/index.md): Hardware assembly and [**Betaflight Configuration**](betaflight.md).
*   [**Infrastructure**](infrastructure/index.md): AWS setup and Docker details.
*   [**Updates & Roadmap**](updates.md): Latest changes and future plans.

---

## ❓ Troubleshooting
If you encounter issues with permissions or missing packages, refer to our [**General Troubleshooting Guide**](controls/troubleshooting.md) or the [**Guidelines**](guidelines.md).
