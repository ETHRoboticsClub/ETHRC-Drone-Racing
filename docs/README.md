# 📖 Documentation Guide

Welcome to the documentation for the ETHRC Drone Racing project. This guide provides structured links to the documentation files stored in the `docs/docs/` folder, enabling easy browsing on GitHub when `mkdocs` is not running.

---

## 🗂 Navigation

Below is a breakdown of the documentation, aligned with our navigation system.

### 🏠 General
- [🏠 Home](docs/index.md)
- [🆕 Recent Updates](docs/updates.md)
- [🚀 Getting Started](docs/getting-started.md)
- [🎮 AIGP Racing Bringup](docs/aigp_bringup.md)
- [⚡ ROS Racing Bringup](docs/racing_bringup.md)
- [📋 Guidelines](docs/guidelines.md)
- [👥 Contributors](docs/contributors.md)

---

### 🔍 Perception
Sensors and state estimation. Each subsystem has a *Technical* page (what it is) and a *Workflow* page (how to run it).

- [Index](docs/perception/index.md) — start here.
- Overview — [Technical](docs/perception/overview/technical.md) · [Workflow](docs/perception/overview/workflow.md)
- Camera — [Technical](docs/perception/camera/technical.md) · [Workflow](docs/perception/camera/workflow.md)
- IMU — [Technical](docs/perception/imu/technical.md) · [Workflow](docs/perception/imu/workflow.md)
- Vicon — [Technical](docs/perception/vicon/technical.md) · [Workflow](docs/perception/vicon/workflow.md)
- VIO — [Technical](docs/perception/vio/technical.md) · [Workflow](docs/perception/vio/workflow.md)
- EKF — [Technical](docs/perception/ekf/technical.md) · [Workflow](docs/perception/ekf/workflow.md)
- Gate detection — [Technical](docs/perception/gate_detection/technical.md) · [Workflow](docs/perception/gate_detection/workflow.md)
- AIGP simulator drivers — [Technical](docs/perception/aigp_drivers/technical.md) · [Workflow](docs/perception/aigp_drivers/workflow.md)
- Calibration — [Technical](docs/perception/calibration/technical.md) · [Workflow](docs/perception/calibration/workflow.md)

---

### 🎛 Controls
Controllers and planners for agile flight.

- [Overview](docs/controls/index.md)
- [Drone Control](docs/controls/drone_control.md) — Main Controller setup.
- [MAVSDK](docs/controls/mavsdk.md) — Communication with PX4.
- [MPC](docs/controls/mpc.md) — Model Predictive Control.
- [RL](docs/controls/rl.md) — Reinforcement Learning.
- [Troubleshooting](docs/controls/troubleshooting.md)

---

### 🔧 Hardware & Infrastructure
- [**Hardware Overview**](docs/hardware/index.md)
  - [IMX219 Tuning](docs/hardware/imx219_90hz_tuning.md)
  - [Drone Connection](docs/hardware/drone_flight_controller_comunication.md)
- [**Groundstation**](docs/groundstation/index.md)

---

<p align="center">
  Proudly developed by the <b>ETH Robotics Club - Aeronautics Division</b>
</p>
