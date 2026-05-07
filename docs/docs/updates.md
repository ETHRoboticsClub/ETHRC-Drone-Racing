# Recent Changes (March 2026)

This month's focus shifted towards ultra-high-speed perception and long-term project scalability. We've established a robust localization pipeline and optimized the onboard sensor stack for competitive drone racing.

---

## ⚡ High-Speed Perception & Field Integration

### IMX219 90Hz High-Speed Tuning
- **Enablement**: Documentation and driver patches for achieving **90Hz/92Hz capture** with a **Full 155° FOV** on the Jetson Orin NX.
- **Optimization**: Modified registration tables and Device Tree timings to prevent CSI FIFO overflows at high frame rates.
- **Guide**: Added [**IMX219 90Hz High-Speed Tuning**](drone-setup/imx219_90hz_tuning.md) as a comprehensive technical resource.

### Vicon Motion Capture Ecosystem
- **Setup**: Detailed instructions and Mermaid wiring diagrams for Vicon hardware deployment.
- **Calibration**: Step-by-step guides for **Dynamic Wand Calibration** and **Static Origin Setup** to ensure sub-centimeter localization accuracy.
- **Documentation**: New [**Vicon Setup Guide**](perception/vicon.md) with clean, non-emoji Mermaid graphs.

---

## 🔄 Firmware & Hardware Evolution

### Custom PX4 v1.15.4 Firmware
- **Multi-Instance**: Developed a custom PX4 build with modified UART mappings to support **three MAVLink instances** simultaneously.
- **Benefit**: Enables parallel high-bandwidth communication for telemetry, VIO feedback, and host PC orchestration.
- **Source**: Check out the `px4/` directory for the pre-built `default.px4` firmware.

### Jetson-to-FC Communication
- **Standardization**: Documented the serial communication protocols and physical wiring required for high-reliability links between the Jetson compute module and the flight controller.

---

## 🏢 Community & Architecture

### System Architecture Visuals
- **Refinement**: Updated the **System Architecture** diagram to reflect the new perception-control data flow, providing a clearer mapping of interfaces between ROS 2 nodes and PX4.

### Team & Governance
- **Contributors**: Added a dedicated [**Project Contributors**](contributors.md) page to recognize the leadership and engineering efforts within the ETHRC Aeronautics division.
- **Structure**: Updated **GitHub Issue Templates** to standardize bug reporting and feature requests.

---

## 🚀 The Road Ahead: Betaflight Migration
- **Decision**: To achieve the next level of agility and latency minimization, the project is officially initiating a **migration towards Betaflight**.
- **Rationale**: Betaflight's specialized focus on racing-optimized PID loops and ultra-low-latency RC protocols provides a superior foundation for competitive autonomous flight.
- **Workflow**: Initial development of the `betaflight_mavlink_bridge` is underway to maintain ROS 2 compatibility while leveraging Betaflight's core flight performance.

---

# Recent Changes (February 2026)

This page summarizes the critical stability improvements and architectural updates recently implemented to ensure a robust development environment.

---

## 🚀 Stability Improvements

### MPC & Acados Compatibility
- **Fixed**: Resolved a breaking `AttributeError` in `drone_opt.py` caused by read-only properties in newer versions of `acados_template`.
- **Solution**: Migrated to environment-based path detection, making the MPC controller compatible with a wider range of Acados installations.
- **Runtime Fix**: Resolved a Python formatting error (`unsupported format string`) in `drone_mpc_controller.py` that occurred when sending goal commands.

### MAVSDK Migration
- **Standardization**: Refactored the drone controller to use standard `mavsdk.offboard` types (`AttitudeRate`, `PositionNedYaw`).
- **Modularity**: Moved the core MAVSDK client into its own ROS 2 package (`my_mavsdk`).

---

## 🐋 Docker & CI Refactoring

- **Hierarchical Build Tree**: Restructured the Dockerfile into a logical dependency tree to maximize layer caching and clean separation:
    - `shared-perception-controls-deps`: Shared root for all functional builds.
    - `shared-mpc-rl-deps`: High-performance math and ML dependencies.
- **Fast Build Targets**:
    - `target: docs`: Now branches directly from `base`, making it the fastest component to build.
    - `target: dev`: Renamed from `full`, seamlessly merges all team-specific layers.
- **Optimized Caching**: The `rl` stage now copies local `mpc` artifacts, cutting down total build time significantly.

### Theme-Aware UI
- **Branding**: Implemented theme-aware SVG logos (ETHRC Aeronautics) that automatically switch between Light and Dark modes.
- **Support**: Enabled full SVG support for documentation logos and favicons.

### CI Build Verification
- **Fixed**: Corrected submodule initialization paths and added `ros2-vicon-receiver` to the automated build verification pipeline.

---

## 📦 Dependency & Submodule Integration

### Mandatory Submodules
- **Integrated**: Properly initialized and updated the `open_vins` (VIO) and `ros2-vicon-receiver` submodules.
- **Build Success**: These components are now fully integrated into the `colcon` build pipeline, ensuring that perception modules work out-of-the-box.

### Permission Handling
- **Streamlined**: Improved the `prepare_colcon_volumes.sh` script and documented its use to resolve common Docker volume permission issues.

---

## 🔍 Visualization & Monitoring

### RViz Integration
- **Feature**: Added native RViz support for the ArUco pose estimation node.
- **Benefit**: Real-time visual feedback of drone pose and marker detections is now accessible via a simple launch argument (`rviz:=true`).

### Orchestration Launch
- **New**: Created/Updated `launch/perception.launch.py` to orchestrate camera bridges, VIO, ArUco, and EKF in a single command.

---

## 🛠️ Internal Cleanup
- Updated all `package.xml` files with correct dependencies.
- Synchronized branch state between `feature/controls` and `perception_merge`.
- Enhanced documentation on [**Getting Started**](getting-started.md) with troubleshooting and visualization guides.
