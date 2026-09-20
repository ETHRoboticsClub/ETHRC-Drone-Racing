# ETHRC Drone Racing

**Page Owner:** [Members of the Aeronautics Division](https://github.com/orgs/ETHRoboticsClub/teams/aeronautics/)

---

<p align="center">
<a href="https://github.com/ETHRoboticsClub/ETHRC-Drone-Racing">
  <img src="images/ETHRC-Aeronautics-Logo-Light.svg" alt="ETHRC Aeronautics Logo" width="70%" class="hero-logo"/>
</a>
</p>

Welcome to the ETHRC Drone Racing project documentation! This repository provides a complete development environment for drone racing research and development.

<p align="center">
  <a href="images/drone_wip.mp4">
    <img src="images/rl_wip.gif" alt="Autonomous RL Training Simulation" width="80%"/>
  </a>
  <br>
  <i>Check out our <b>Autonomous RL Training Simulation</b> (click above to play)</i>
</p>

Everyone in the project is free to commit changes to the docs and we really encourage you to do so!
If you are unsure whether the information belongs here or on our Notion page, then feel free to ask.
Relevant technical details (implementation, designs, etc.) should live here.
The project has recently migrated to **Betaflight** as the primary flight controller firmware for hardware deployments to achieve superior racing performance.

## 🏗 System Architecture

The project integrates high-fidelity simulation with advanced sensing and control for autonomous racing.

<p align="center">
  <img src="images/Interfaces.drawio.png" width="90%" alt="System Architecture" />
</p>

## What's New? (Feb 2026)

- **Hierarchical Build Tree**: Restructured Docker for faster builds and better caching (merging `dev` target).
- **Theme-Aware UI**: Added light/dark mode support for project logos and documentation branding.
- **Enhanced Perception**: Correctly integrated `open_vins` and `ros2-vicon-receiver` as submodules.
- **Betaflight Migration**: Successfully transitioned to Betaflight/MSP for low-latency autonomous control.
- **Visualization Suite**: New instructions for RViz and Gazebo monitoring are now available in the [**Perception Overview**](perception/index.md#visualization-and-monitoring).

**[Read the full Updates & Roadmap here &raquo;](updates.md)**

## Quick Links

- [**Getting Started**](getting-started.md): Set up your environment and run your first simulation.
- [**Guidelines**](guidelines.md): Learn about our documentation standards and development workflow.
- [**Perception**](perception/index.md): Documentation for the Perception team and modules.
- [**Controls**](controls/index.md): Documentation for the Controls team and algorithms.
- [**Drone Setup**](drone-setup/index.md): Documentation for the hardware, electronics, and onboard setup.
- [**Groundstation**](groundstation/index.md): Documentation for the Groundstation team and software.

## Project Layout

The project is organized as follows:

- **`docker/`**: Configuration for the Docker environment.
- **`px4/`**: PX4 Autopilot source code submodule.
- **`docs/`**: Documentation source files (MkDocs).
    - **`docs/`**: Markdown content.
    - **`mkdocs.yml`**: Site configuration.
- **`src/`**: ROS 2 packages and source code.
    - **`perception/`**: Perception modules and algorithms.
    - **`utils/`**: Utility packages.
- **`launch/`**: Global launch files for the project (e.g., `perception.launch.py`).
- **`scripts/`**: Helper scripts for setup and running simulations.
- **`sim/`**: Simulation resources, including Gazebo worlds and models.
- **`docker-compose.yaml`**: Docker Compose definition for services and volumes.

## Getting Started

Check out the `README.md` in the root of the repository or [**Getting Started**](getting-started.md) for instructions on how to build and run the simulation.

## Subsystems
Find the following subsystems:

- [**Perception**](perception/index.md): Documentation for the Perception team and modules.
- [**Controls**](controls/index.md): Documentation for the Controls team and algorithms.
- [**Drone Setup**](drone-setup/index.md): Documentation for the hardware, electronics, and onboard setup.
- [**Groundstation**](groundstation/index.md): Documentation for the Groundstation team and software.
