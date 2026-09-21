# Project Updates & Roadmap

**Page Owner:** [Dimetri Chau](https://github.com/deonixlive)

This page summarizes recent changes to the ETHRC Drone Racing project and outlines our future roadmap for development.

---

## 📅 Recent Changes (2026)

### ⚡ High-Speed Perception & Field Integration
- **IMX219 90Hz High-Speed Tuning**: Documentation and driver patches for achieving **90Hz/92Hz capture** with a **Full 155° FOV** on the Jetson Orin NX.
- **Vicon Motion Capture Ecosystem**: Detailed instructions and wiring diagrams for Vicon hardware deployment, including dynamic wand calibration.
- **Documentation**: New [**Vicon Setup Guide**](perception/vicon/workflow.md) and [**IMX219 90Hz Tuning**](drone-setup/imx219_90hz_tuning.md).

### 🧪 Boreas Betaflight SITL Control Model

- Added a shared Boreas 4S airframe profile backed by the CAD-derived URDF.
- Added motor dynamics, reaction torque, body drag, and plant diagnostics.
- Added automated plant, RL, and MPC SITL validation reports.
- Corrected QUADX rotor order, the Betaflight IMU mount, ROS angular-rate
  conversion, and the controller-to-Betaflight pitch convention.
- Added a one-shot virtual launch restraint to remove physical-pad contact
  impulses from control-policy evaluation.
- Made MPC invert the shared quadratic thrust curve when converting force in
  newtons to Betaflight throttle.
- Added the measured Boreas maximum of 916 gf per propeller and updated the
  derived 4.387:1 thrust-to-weight and 0.4774 hover-command baselines.
- Aligned RL/SITL with the real linear 500 deg/s Boreas rate profile and
  hardware PID profile. The resulting hardware-faithful RL hover test currently
  fails, exposing a policy transfer issue rather than masking it with caps.
- Corrected MPC from the legacy X500 mass and made `controller_type:=mpc`
  functional. Propulsion and drag remain provisional pending physical tests.

### 🔄 Firmware & Hardware Evolution
- **Betaflight Migration**: The project has officially migrated to **Betaflight** as the primary flight controller firmware. This provides superior PID loops and lower latency for competitive racing.
- **Custom PX4 v1.15.4**: (Legacy) Developed custom build with modified UART mappings for multi-instance MAVLink.
- **Jetson-to-FC Communication**: Standardized serial protocols for high-reliability links between Jetson and the FC.

### 🏢 Community & Architecture
- **System Architecture**: Updated diagrams reflecting the perception-control data flow and ROS 2 node interfaces.
- **Project Governance**: Added a [**Contributors**](contributors.md) page and standardized GitHub Issue templates.

---

## 🛠️ Development Workflow Roadmap

### 1. Docker Build & Volume Strategy Optimization
*   **Goal:** Decouple environment dependencies from workspace compilation to improve developer velocity.
*   **Dev Container:** Shift towards a container that only installs system dependencies (ROS base, apt packages, pip dependencies, ACADOS). Source code and build directories should be mounted as volumes.
*   **Release Container:** Implement a `target: release` stage that bakes the code and compiles it for minimal image size, used for CI testing and deployment to Jetson Orin NX.

### 2. Automated Testing Pipeline (CI/CD)
*   **Goal:** Ensure code reliability and prevent regressions.
*   **Unit Tests**: Integrate `colcon test` into GitHub Actions.
*   **Integration Tests**: Implement **ROS 2 Launch Testing** (`launch_testing`) to verify node communication (MPC, EKF, Perception) in a headless SITL environment.

### 3. Static Analysis & Linting
*   **Goal:** Maintain high code quality and style consistency.
*   **Pre-commit Hooks**: Implement a framework to enforce standards (C++: `clang-format`, Python: `ruff`/`black`).

---

## 🏎️ Autonomous Drone Racing Techniques

### 1. Perception & Sim-to-Real Transfer
*   **Domain Randomization**: Implement lighting, texture, and noise randomization in Gazebo to better prepare ML models for real-world FPV camera characteristics.
*   **Robust Gate Tracking**: Move to a **Tracking-by-Detection** pipeline (e.g., YOLOv8 + Optical Flow + Kalman Filter).

### 2. Advanced State Estimation
*   **Factor Graphs**: Transition from EKF to **Factor Graph optimization (GTSAM)** to better handle out-of-sequence measurements and delayed VIO.

### 2. Control & Trajectory Planning
*   **MPC Stability & Performance**:
    *   Implement explicit failure handling for solver divergence.
    *   Ensure Real-Time (RT) execution by bounding solution time.
    *   Refactor model building to avoid redundant recompilation during runtime.
*   **Time-Optimal Trajectory Generation**: Compute physical limits of the track using **CPC (Complementary Progress Control)** or NLP solvers (CasADi).
*   **Aerodynamic Drag Modeling**: Integrate high-fidelity quadratic drag models into the ACADOS MPC formulation.
*   **Residual Reinforcement Learning**: Combine MPC reliability with RL adaptability by training policies to output control corrections for unmodeled dynamics.
