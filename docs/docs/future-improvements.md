# Future Improvements Roadmap

This page outlines potential enhancements for the ETHRC Drone Racing project, focusing on development workflow efficiency and cutting-edge autonomous racing techniques.

## 🛠️ Development Workflow Improvements

### 1. Docker Build & Volume Strategy Optimization
*   **Goal:** Decouple environment dependencies from workspace compilation to improve developer velocity.
*   **Dev Container:** Shift towards a container that only installs system dependencies (ROS base, apt packages, pip dependencies, ACADOS). Source code and build directories should be mounted as volumes.
*   **Release Container:** Implement a `target: release` stage that bakes the code and compiles it for minimal image size, used for CI testing and deployment to Jetson Orin NX.

### 2. Automated Testing Pipeline (CI/CD)
*   **Goal:** Ensure code reliability and prevent regressions.
*   **Unit Tests:** Integrate `colcon test` into GitHub Actions.
*   **Integration Tests:** Implement **ROS 2 Launch Testing** (`launch_testing`) to verify node communication (MPC, EKF, Perception) in a headless SITL environment.
*   **Mocked Tests:** Verify MPC logic using artificial VIO data streams.

### 3. Static Analysis & Linting
*   **Goal:** Maintain high code quality and style consistency.
*   **Pre-commit Hooks:** Implement a framework to enforce standards before code is pushed.
    *   **C++:** `clang-format`, `cpplint`, or `clang-tidy`.
    *   **Python:** `ruff` or `black` + `flake8`.

---

## 🏎️ Autonomous Drone Racing Techniques

### 1. Perception & Sim-to-Real Transfer
*   **Domain Randomization:** Implement lighting, texture, and noise randomization in Gazebo to better prepare ML models for real-world FPV camera characteristics.
*   **Advanced Rendering:** Explore **NeRFs (Neural Radiance Fields)** or **Gaussian Splatting** for photorealistic simulation environments.
*   **Robust Gate Tracking:** Move beyond single-frame detection to a **Tracking-by-Detection** pipeline (e.g., YOLOv8 + Optical Flow + Kalman Filter) to handle high-speed motion blur.

### 2. Advanced State Estimation
*   **Factor Graphs:** Transition from EKF to **Factor Graph optimization (GTSAM)**. This better handles:
    *   Out-of-sequence measurements.
    *   Delayed visual odometry.
    *   High-frequency IMU data synchronization.

### 3. Control & Trajectory Planning
*   **Time-Optimal Trajectory Generation:** Compute the absolute physical limits of the track using **CPC (Complementary Progress Control)** or NLP solvers (CasADi) to create dynamically feasible, time-optimal references for the MPC.
*   **Aerodynamic Drag Modeling:** Integrate a high-fidelity quadratic drag model into the ACADOS MPC formulation to account for dominant forces at high speeds.
*   **Residual Reinforcement Learning:** Combine MPC reliability with RL adaptability by training policies to output small control corrections ("residuals") to account for unmodeled dynamics like battery sag and downwash.
