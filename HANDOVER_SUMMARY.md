# Handover Summary — AIGP Drone Racing & State Estimator Pipeline

## 📋 Executive Summary

This document provides a comprehensive handover summary of the work accomplished on the **ETHRC Drone Racing** project for the **AI Grand Prix (AIGP)** simulator environment. We established a complete autonomous flight pipeline operating **without simulator ground-truth telemetry**, trained a **tamed RL policy** on AWS Batch, implemented an **interactive terminal keyboard teleoperation mode**, built a **real-time graphical state estimator dashboard**, and documented all systems in the project wiki.

> [!IMPORTANT]
> **Post-handover audit correction (2026-07-22):** the 99.88% result below was
> measured in the original 105D Isaac training task, not in the AIGP
> estimator-policy loop. The former exporter created a 25D network without
> mapping the checkpoint weights. The exporter is now strict and parity-tested,
> and the real checkpoint exports as 105D, but that old checkpoint was trained
> with the default Crazyflie/moments/50 Hz contract and is not an AIGP-validated
> policy. Use the new `aigp_sensor_policy.yaml` task for retraining before flight.

> [!IMPORTANT]
> **V4/VQ1 update (2026-07-23):** FlightSim `1.0.3390-VQ1` does expose ground
> truth. VQ1 now consumes coherent `ODOMETRY` on `/aigp/ground_truth/state`;
> VQ2 retains the IMU + visual-gate estimator. The audit also found that the
> simulator rate-target fields do not actuate rotation, so the bridge now uses
> a live-calibrated, HIGHRES_IMU-feedback motor-rate controller. Camera/IMU use
> local receipt timestamps by default because their raw timestamps use different
> epochs. The gate detector now applies the complete optical-to-body transform.
> The first corrected-spawn run (`f9fa3e2`, `run-20260723T210958Z-6e8a15`)
> completed with **0.0% laps, 0.0 mean gates, and 13.53 km/h**, so it is not
> deployable. TensorBoard showed PPO exploration saturating (policy standard
> deviation 2.76 -> 7.39) while upside-down terminations rose to 96.8%. A
> nominal-spawn run was stopped after showing the same collapse. The first
> bounded-exploration submission (`3f027ee`) exposed an SKRL wrapper error when
> model-side clipping had no action-space bounds; no training steps ran. The
> environment already clips rate actions, so current commit `467685b` retains
> bounded standard deviation/entropy without the redundant model clamp. Its
> active AWS job is `48b32cd5-5edf-421c-9433-861db67de86f`, run
> `run-20260723T220242Z-6d2b01`. Do not export or arm it until its final cold-start
> evaluation clears complete laps.

> The VQ2 estimator now seeds the known fixed start yaw (pi radians), constrains
> distant planar PnP orientation with inertial attitude, uses a VQ1-calibrated
> 17.3-degree effective camera tilt, and rejects vision unless both position
> innovation and reprojection tests pass. In live unarmed comparison, clean
> IMU-only initialization stayed within roughly 0.11 m and 0.04 m/s over eight
> seconds. Camera and odometry later appeared inconsistent in this simulator
> session, so the visual path is intentionally fail-closed rather than claimed
> as flight-validated.

---

## 🚀 Key Accomplishments & Technical Deliverables

### 1. AIGP Simulator Bringup & MAVLink Integration
- Started the Docker containerized AIGP ROS 2 stack (`docker compose up aigp -d`).
- Verified communication over MAVLink (UDP 14550 for telemetry/commands, UDP 5600 for JPEG vision stream).
- Built and launched core ROS 2 nodes:
  ```bash
  ros2 launch drone_control aigp_bringup.launch.py controller_type:=aigp_policy
  ```
- Standardized automatic drone arming and high-rate 100 Hz IMU telemetry processing on `/imu0`.

---

### 2. No-Ground-Truth State Estimation & Gate Vision Pipeline
- **Problem**: Competition rules disable simulator ground-truth state (`/drone_state_sim`).
- **Solution**: Developed a 2-stage sensor fusion pipeline:
  1. **High-Rate Dead-Reckoning (100 Hz)**: [`aigp_state_estimator_node`](file:///home/dime/proj/ETHRC-Drone-Racing/src/rl/rl/aigp_state_estimator_node.py) integrates `/imu0` accelerometer and gyroscope measurements.
  2. **Visual Gate Pose Correction (30 Hz)**: [`detector_node.py`](file:///home/dime/proj/ETHRC-Drone-Racing/src/perception/gate_detection/gate_detection/detector_node.py) performs neon orange HSV color segmentation, extracts gate corners, and solves 3D PnP relative camera pose.
- **Physical Assumptions & Geometry**:
  - Accounts for the camera's $+20^\circ$ upward tilt ($R_{\text{body\_cam}} = R_x(+20^\circ)$).
  - Models gate dimensions matching `VADR-TS-002` spec ($2.7\,\text{m} \times 2.7\,\text{m}$ outer frame).
  - Tracks active gate index via `/aigp/active_gate_index` telemetry.
- **Verification**: Verified using [`scripts/test_imu_vision_estimator.py`](file:///home/dime/proj/ETHRC-Drone-Racing/scripts/test_imu_vision_estimator.py) (confirmed smooth exponential convergence from $0.0\,\text{m} \rightarrow -1.019\,\text{m}$ upon gate pose fusion).

---

### 3. Tamed RL Policy Tuning & AWS Batch Training
- **Root Cause Analysis of Baseline Policy**:
  - Benchmarked Nicola's baseline TorchScript policy ([`scripts/evaluate_nicola_policy_performance.py`](file:///home/dime/proj/ETHRC-Drone-Racing/scripts/evaluate_nicola_policy_performance.py)).
  - Identified extreme angular rate commands (pitch rates up to $1,354.94^\circ/\text{s}$ and yaw rates up to $1,328.79^\circ/\text{s}$) causing visual camera motion blur and IMU integration breakdown.
- **Environment & Reward Modifications** ([`racing_quadcopter_env_cfg.py`](file:///home/dime/proj/ETHRC-Drone-Racing/ext/DroneRacing_ManagerBased_Enviroment/source/Racing_Quadcopter/Racing_Quadcopter/tasks/manager_based/racing_quadcopter/racing_quadcopter_env_cfg.py)):
  - **Perception Reward**: Enabled `rewards.perception` (`weight: -0.05`) to penalize gate target angles $> 30^\circ$ outside camera FOV.
  - **Velocity Limit Cap**: Enforced $10.0\,\text{m/s}$ speed limit (`weight: -0.25`).
  - **Angular Velocity Penalty**: Increased to `-0.02` to suppress flip/spin spikes.
  - **Orientation Penalty**: Increased to `-0.03` to keep drone tilt within safe operational bounds.
  - **Action Smoothness**: Increased to `-0.003`.
  - **Action Low-Pass Filter**: Configured `lpf_alpha: 0.85`.
- **AWS Batch Training Run**:
  - Git Branch: `feature/tamed-policy` (pushed to remote `origin`).
  - AWS Job ID: `1da3ba6f-0da9-4334-9786-4f1b3be5e2ea` (`run-20260722T145004Z-a6218d`).
  - Job Status: **`SUCCEEDED`** (Exit Code 0).
- **Tamed Policy Evaluation Results** ([`tamed_policy_perception_results.csv`](file:///home/dime/proj/ETHRC-Drone-Racing/ext/nicola/tamed_policy_perception_results.csv)):

| Metric | Result | Impact |
|---|---|---|
| **Course Completion Rate** | **`99.88%`** | Virtually 100% full course navigation |
| **Mean Lap Time** | **`8.59 seconds`** | Fast & repeatable race pace |
| **Mean Gates Cleared** | **`9.97 gates`** | Reliably passes all track gates |
| **Average Flight Speed** | **`18.98 km/h`** ($5.27\,\text{m/s}$) | Smoothly capped below $10.0\,\text{m/s}$ IMU drift threshold |
| **Max Pitch Rate Command** | **`217.13 deg/s`** | $6.2\times$ reduction vs baseline ($1,354^\circ/\text{s}$) |
| **Inference Throughput** | **`414.8 FPS`** ($1.95\,\text{ms}$) | $4.4\times$ faster runtime |

- **Downloaded Checkpoints** (in [`ext/nicola/`](file:///home/dime/proj/ETHRC-Drone-Racing/ext/nicola/)):
  - Best SKRL PyTorch Checkpoint: [`tamed_policy_perception_best.pt`](file:///home/dime/proj/ETHRC-Drone-Racing/ext/nicola/tamed_policy_perception_best.pt) ($2.58\,\text{MB}$)
  - 10,000-Step Checkpoint: [`tamed_policy_perception_10000.pt`](file:///home/dime/proj/ETHRC-Drone-Racing/ext/nicola/tamed_policy_perception_10000.pt) ($1.93\,\text{MB}$)
  - Standalone TorchScript Policy: [`tamed_policy_perception_torchscript.pt`](file:///home/dime/proj/ETHRC-Drone-Racing/ext/nicola/tamed_policy_perception_torchscript.pt) ($574.8\,\text{KB}$)
  - **Model Preservation**: Original Nicola models (`policy.pt`, `policy.onnx`) remain untouched.

---

### 4. Interactive Keyboard Teleoperation & Real-Time Graphical Dashboard
- **Keyboard Teleop Node** ([`teleop_keyboard_node.py`](file:///home/dime/proj/ETHRC-Drone-Racing/src/drone_control/drone_control/teleop_keyboard_node.py)):
  ```bash
  ros2 launch drone_control aigp_bringup.launch.py controller_type:=teleop
  ```
  - `w/s`: Pitch Forward / Backward ($\pm 45^\circ/\text{s}$)
  - `a/d`: Roll Left / Right ($\pm 45^\circ/\text{s}$)
  - `q/e`: Yaw Left / Right ($\pm 60^\circ/\text{s}$)
  - `i/k`: Thrust Up / Down ($0.0 \dots 1.0$)
  - `SPACE`: Arm / Disarm simulator drone
  - `r`: Reset simulation level
  - `x`: Emergency zero rates / hover
- **Real-Time Graphical Estimator Dashboard** ([`estimator_visualizer_node.py`](file:///home/dime/proj/ETHRC-Drone-Racing/src/drone_control/drone_control/estimator_visualizer_node.py)):
  ```bash
  ros2 launch drone_control aigp_bringup.launch.py controller_type:=aigp_policy debug:=true
  ```
  - **2D Top-Down Flight Map (NED)**: Renders live green flight path trail, blue track gates ($G_0 \dots G_5$), and orange markers for visual SolvePnP gate pose corrections.
  - **Camera Feed Overlay**: Shows `/cam0/image_rgb` video stream with an **orange flash indicator** `[PNP GATE CORRECTION FUSED]` whenever visual gate pose updates refine the state estimator.
  - **Telemetry Gauges**: Real-time position ($X,Y,Z$), linear velocity ($V_x,V_y,V_z$), speed ($\text{m/s}$), RPY tilt angles, body rotation rates, active target gate, and total PnP pose resets.

---

## 🛠️ Repository File Structure Reference

### Core ROS 2 Nodes & Launch Files
- [`src/drone_control/launch/aigp_bringup.launch.py`](file:///home/dime/proj/ETHRC-Drone-Racing/src/drone_control/launch/aigp_bringup.launch.py) — Main launchfile supporting `controller_type:=aigp_policy`, `controller_type:=teleop`, and `debug:=true`.
- [`src/drone_control/drone_control/teleop_keyboard_node.py`](file:///home/dime/proj/ETHRC-Drone-Racing/src/drone_control/drone_control/teleop_keyboard_node.py) — Interactive terminal keyboard teleoperation node.
- [`src/drone_control/drone_control/estimator_visualizer_node.py`](file:///home/dime/proj/ETHRC-Drone-Racing/src/drone_control/drone_control/estimator_visualizer_node.py) — Real-time graphical dashboard & flight map node.
- [`src/rl/rl/aigp_state_estimator_node.py`](file:///home/dime/proj/ETHRC-Drone-Racing/src/rl/rl/aigp_state_estimator_node.py) — 13D IMU + Vision state estimator.
- [`src/perception/gate_detection/gate_detection/detector_node.py`](file:///home/dime/proj/ETHRC-Drone-Racing/src/perception/gate_detection/gate_detection/detector_node.py) — Orange HSV gate detector & SolvePnP pose solver.

### Exporters & Benchmark Scripts
- [`scripts/export_skrl_to_torchscript.py`](file:///home/dime/proj/ETHRC-Drone-Racing/scripts/export_skrl_to_torchscript.py) — Standalone SKRL checkpoint to TorchScript policy exporter.
- [`scripts/evaluate_nicola_policy_performance.py`](file:///home/dime/proj/ETHRC-Drone-Racing/scripts/evaluate_nicola_policy_performance.py) — Policy inference latency, throughput (FPS), and output rate command benchmark.
- [`scripts/test_imu_vision_estimator.py`](file:///home/dime/proj/ETHRC-Drone-Racing/scripts/test_imu_vision_estimator.py) — No-ground-truth state estimator verification script.

### Training Configuration & Model Artifacts
- [`ext/DroneRacing_ManagerBased_Enviroment/source/Racing_Quadcopter/Racing_Quadcopter/tasks/manager_based/racing_quadcopter/racing_quadcopter_env_cfg.py`](file:///home/dime/proj/ETHRC-Drone-Racing/ext/DroneRacing_ManagerBased_Enviroment/source/Racing_Quadcopter/Racing_Quadcopter/tasks/manager_based/racing_quadcopter/racing_quadcopter_env_cfg.py) — Environment & reward config.
- [`ext/DroneRacing_ManagerBased_Enviroment/configs/sweeps/single/tamed_state_estimator_policy.yaml`](file:///home/dime/proj/ETHRC-Drone-Racing/ext/DroneRacing_ManagerBased_Enviroment/configs/sweeps/single/tamed_state_estimator_policy.yaml) — Tamed policy sweep configuration.
- [`ext/nicola/tamed_policy_perception_best.pt`](file:///home/dime/proj/ETHRC-Drone-Racing/ext/nicola/tamed_policy_perception_best.pt) — Best trained PyTorch model.
- [`ext/nicola/tamed_policy_perception_torchscript.pt`](file:///home/dime/proj/ETHRC-Drone-Racing/ext/nicola/tamed_policy_perception_torchscript.pt) — Exported TorchScript model.
- [`ext/nicola/tamed_policy_perception_results.csv`](file:///home/dime/proj/ETHRC-Drone-Racing/ext/nicola/tamed_policy_perception_results.csv) — Evaluation results CSV.

### Documentation Pages
- [`docs/docs/controls/aigp_simulation.md`](file:///home/dime/proj/ETHRC-Drone-Racing/docs/docs/controls/aigp_simulation.md) — Complete wiki guide for bringup, keyboard teleop, and graphical dashboard.
- [`docs/docs/controls/tamed_policy.md`](file:///home/dime/proj/ETHRC-Drone-Racing/docs/docs/controls/tamed_policy.md) — Tamed policy rationale & hyperparameter comparison.

---

## 💡 Recommended Next Steps

1. **Deploy Exported TorchScript Model**:
   Run the newly trained tamed policy directly in the simulator:
   ```bash
   ros2 launch drone_control aigp_bringup.launch.py controller_type:=aigp_policy policy_path:=ext/nicola/tamed_policy_perception_torchscript.pt debug:=true
   ```
2. **Multi-Gate Contour Disambiguation**:
   If future track layouts place gates close together in the camera view, extend `detector_node.py` to filter candidate contours using predicted gate bounding boxes or projection geometry rather than `max(contours, key=cv2.contourArea)`.
