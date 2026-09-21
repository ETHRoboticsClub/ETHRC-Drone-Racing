<p align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="docs/docs/images/ETHRC-Aeronautics-Logo-Dark.svg">
    <source media="(prefers-color-scheme: light)" srcset="docs/docs/images/ETHRC-Aeronautics-Logo-Light.svg">
    <img alt="ETHRC Aeronautics" src="docs/docs/images/ETHRC-Aeronautics-Logo-Light.svg" width="600">
  </picture>
</p>

<p align="center">
  <b>Pushing the boundaries of Autonomous Drone Racing with ROS 2, PX4, and Cutting-edge Perception.</b>
</p>

<p align="center">
  <img src="https://img.shields.io/badge/ROS2-Humble-blue?style=flat-square&logo=ros" alt="ROS2 Humble"/>
  <img src="https://img.shields.io/badge/Simulator-Gazebo-orange?style=flat-square&logo=gazebo" alt="Gazebo"/>
  <img src="https://img.shields.io/badge/Autopilot-PX4-blue?style=flat-square" alt="PX4"/>
  <img src="https://img.shields.io/badge/Docker-Supported-2496ED?style=flat-square&logo=docker" alt="Docker"/>
</p>

---

## 🚀 The Mission

Welcome to the **ETHRC Drone Racing** project. We are building the next generation of autonomous racing drones capable of navigating complex, high-speed environments with surgical precision. This repository provides a complete, containerized development environment - from high-fidelity simulation to real-world hardware deployment.

![Drone in Action](docs/docs/images/drone_action_front.webp)

---

## 🛠 Project Pillars

| **Perception** | **Control** | **Simulation** |
| :--- | :--- | :--- |
| **State Estimation:** VIO (OpenVINS) + EKF for robust localization. | **High-Speed Tracking:** MPC and RL-based controllers for agile flight. | **Digital Twin:** Gazebo Harmonic for realistic physics and sensor noise. |
| **Object Detection:** Real-time gate and ArUco marker tracking. | **Planning:** Minimal-time trajectory generation through race gates. | **SITL:** PX4 Software-In-The-Loop for autopilot parity. |

---

## 🛰️ Drone Setup

We don't just simulate; we fly. Our hardware stack is designed for extreme performance and modularity. We have multiple drone configurations named after the Greek wind gods.

### 🌬️ Zephyrus (6S Performance)
*6S drone with a Kakute H7 flight controller.*

*   **Hardware & Parts List**: [TBD]
*   **Soldering & Connections**: [TBD]
*   **Betaflight Configuration**: [TBD]
*   **Specifics**: [TBD]

### 🌬️ Boreas (4S Agile)
*4S drone with a JHEMCU generic FC (H743 chip).*

*   **Hardware & Parts List**: [TBD]
*   **Soldering & Connections**: [TBD]
*   **Betaflight Configuration**: [TBD]
*   **Specifics**: [TBD]

<p align="center">
  <img src="docs/docs/images/drone_hardware_closeup.webp" width="45%" />
  <!-- <img src="docs/docs/images/drone_lab_arena.webp" width="45%" /> -->
</p>

<p align="center">
  <img src="docs/docs/images/rl_wip.gif" width="45%" />

  <i><b>Autonomous RL Training Simulation</b>: Drones learning optimal racing lines in IsaacLab.</i>
</p>

---

## ⚡ Quick Start

Get your environment up and running in minutes.

### 1. Clone & Initialize
```bash
git clone --recurse-submodules git@github.com:ETHRoboticsClub/ETHRC-Drone-Racing.git
cd ETHRC-Drone-Racing
```

### 2. Launch Environment
We use Docker to ensure "it works on everyone's machine." Before building the images and running it, make sure to run the AWS setup script. This enables you to pull prebuild images, where you don't have to compile the heavy dependencies yourself and saves you a lot of time.

```bash
# From the root of the repo, run the AWS setup script
./scripts/tools/setup_aws_ecr.sh

# Prepare host volumes (Fixes 'Permission Denied' errors in Docker)
./scripts/prepare_colcon_volumes.sh

# Start the development container
xhost +local:root
docker compose up dev -d
docker compose exec dev bash
```

> **Low-RAM machine / PC freezes during the build?** Cap the compile parallelism (default is all cores) and build the stages one at a time:
> ```bash
> export BUILD_JOBS=4
> docker compose build perception && docker compose build mpc && docker compose build dev
> ```

### 🍏 macOS Users: One-Time Setup

The Docker services in this repo use `network_mode: host`. On Linux this works out of the box, but on **macOS** Docker Desktop runs containers inside a Linux VM, so host networking is **disabled by default** — ports like `http://localhost:8000` (docs), and any other service port, will not be reachable from your Mac browser even when the container is running and serving correctly.

You only need to do this once:

1.  Open **Docker Desktop → Settings → Resources → Network**.
2.  Check **“Enable host networking”**.
3.  Click **Apply & restart**.

Requires **Docker Desktop 4.34 or newer**. After enabling, all host-networked services in `docker-compose.yaml` (`dev`, `docs`, `perception`, `mpc`, `rl`) behave the same on macOS as on Linux — no changes to the compose file needed.

> **If the “Enable host networking” checkbox is missing**, sign in to your Docker account in Docker Desktop (free account) and reopen the Network settings. Docker gates the option behind sign-in for some users.

To verify, after starting the docs service you should get `HTTP/1.0 200 OK`:
```bash
docker compose up docs -d
curl -sI http://localhost:8000/
```

### 3. Build & Fly
Inside the container:
```bash
# Build the workspace
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash

# Launch Simulation & Controller
# IMPORTANT: Run 'source install/setup.bash' in every NEW terminal!
ros2 launch drone_control drone_controller.launch.py
```

---

## 🏗 System Architecture

Our system is composed of several high-performance subsystems working in harmony. The perception pipeline feeds high-frequency state updates to the MPC controller, ensuring agile and precise flight.

![System Architecture](docs/docs/images/Interfaces.drawio.png)

---
## ⚡ High-Rate Betaflight IMU & Control

We support 2kHz IMU streaming and low-latency autonomous control via a dedicated dual-port architecture:
-   **Telemetry (UART)**: 1.5 Mbps stream from Betaflight to `blackbox_driver_node` for high-rate VIO (OpenVINS).
-   **Control (USB)**: Bidirectional MSP bridge (`msp_bridge`) for RC simulation and attitude command injection.
-   **Sync**: Hardware Timestamping Engine (HTE) on Jetson Pin 16 for microsecond-accurate alignment.

### Quick Deploy
```bash
# Launch the full real-world stack on the Jetson
ros2 launch drone_control racing_bringup.launch.py
```
For detailed hardware wiring and CLI configuration, see [docs/docs/betaflight.md](docs/docs/betaflight.md).

---

## 🏗 Hardware-Specific Build Notes

### Jetson Orin NX (Zephyrus, Boreas)
The simulation stack (Gazebo Harmonic and the ROS 2 Gazebo bridge) is conditionally skipped when building on `arm64` (Jetson) to optimize build time and avoid missing architecture-specific binary dependencies. Onboard systems for perception, control, and MAVSDK are still fully built and supported.

## 🤝 Join the Race

Are you passionate about robotics, computer vision, or control theory? We are always looking for motivated engineers to join the ETHRC Aeronautics division.

- 📖 **Documentation:** [http://localhost:8000](http://localhost:8000) (if running locally via `docker compose up docs -d`) or [https://racing.deonix.org](https://racing.deonix.org) (access protected, credentials shared internally)
- 📝 **Guidelines (with Coding Agents):** Check out `aeronautics-utility/agents/AGENT.md` for our coding standards.
- 💬 **Get Involved:** Reach out to the team on Discord or Open an Issue!

---

<p align="center">
    Proudly developed by the <b>ETH Robotics Club - Aeronautics Division</b>
</p>

---
