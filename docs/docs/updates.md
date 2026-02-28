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
