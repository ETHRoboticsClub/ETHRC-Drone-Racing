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

## Native Execution (Non-Docker)

While Docker is great for simulation and development on a workstation, you may want to run part of the stack **natively** on your companion computer (e.g., Jetson Orin NX) to reduce overhead or access specific hardware pins.

### 1. Prerequisites
Ensure you have ROS 2 installed on your host system (this project targets **Humble**, though it is compatible with **Jazzy**). 
```bash
# Example for installing colcon
sudo apt update && sudo apt install python3-colcon-common-extensions
```

### 2. Build the Workspace
From the project root:
```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 3. Fast Bringup (The "Big Button")
Once built, you can launch the entire racing stack across all modules (Comms, Perception, and Control) using the unified bringup script:

```bash
source install/setup.bash
ros2 launch drone_control racing_bringup.launch.py controller_type:=rl
```

#### Bringup Arguments

| Argument | Default | Description |
| :--- | :--- | :--- |
| `controller_type` | `rl` | Choice of high-level controller: `rl` or `mpc`. |
| `rate_error_scale`| `300.0`| **(RL Only)** Scaling factor to map neural network torques to Betaflight attitude rates. |
| `msp_port` | `/dev/ttyTCU0`| Serial port for Betaflight MSP control (dedicated port). |
| `imu_port` | `/dev/ttyTHS1`| Serial port for Blackbox high-rate IMU telemetry. |
| `use_vicon` | `true` | Whether to launch the Vicon receiver for ground truth. |
| `use_rc_sim` | `true` | Whether to simulate an RC receiver for arming via MSP. |
| `view_stream` | `true` | Start the video decoding/republishing pipeline (GS only). |

### 4. Configuration Tips
- **Serial Ports:** If you are using a USB-to-Serial adapter instead of the internal UART, use `msp_port:=/dev/ttyUSB0`.
- **RL Tuning:** If the drone feels sluggish under RL control, increase `rate_error_scale` (e.g., to `500.0`). If it oscillates, decrease it.
- **Persisting Settings:** You can edit the default parameters in `src/drone_control/config/drone_controller_config.yaml` so you don't have to pass them every time.

---

## ❓ Troubleshooting
If you encounter issues with permissions or missing packages, refer to our [**General Troubleshooting Guide**](controls/troubleshooting.md) or the [**Guidelines**](guidelines.md).
