# AIGP Simulator

**Page Owner:** [Members of the Aeronautics Division](https://github.com/orgs/ETHRoboticsClub/teams/aeronautics/)

---

The **AI Grand Prix (AIGP)** simulator is our virtual qualification environment for autonomous racing. It uses its own MAVLink and UDP interfaces rather than the normal PX4/MAVSDK path, so this repository provides a dedicated bridge, camera driver, state-estimation pipeline, and controllers.

!!! success "VQ1 completion status"
    The ground-truth VQ1 controller completed the six-gate competition track in
    FlightSim `1.0.3390-VQ1`. The validated run used a deliberately conservative
    target speed of `1.0 m/s`, passed all six gates, and produced the simulator's
    race-finish event in approximately **165 seconds**.

| Qualification | State source | Recommended controller | Current status |
| --- | --- | --- | --- |
| **VQ1** | Simulator `ODOMETRY` ground truth | `aigp_vq1_gt` | **Six-gate lap validated** |
| **VQ2** | IMU + visual gate detections | `aigp_policy` + ESKF | Estimation pipeline implemented; full lap not yet validated |

## Quick start: complete VQ1

Use this path when the immediate goal is to complete VQ1. Start FlightSim on the host first, then run:

```bash
cd /path/to/ETHRC-Drone-Racing

colcon build --packages-select \
  rl betaflight_mavlink_bridge drone_control

source install/setup.bash
export ROS_LOCALHOST_ONLY=1

ros2 launch drone_control aigp_bringup.launch.py \
  controller_type:=aigp_vq1_gt \
  vq1_controller_enabled:=true \
  rate_control_backend:=actuator
```

The launch defaults are the settings used for the completed lap. In particular, the controller targets `1.0 m/s`; speed is intentionally secondary to reliable gate passage.

!!! warning "Only one command source"
    Do not run teleoperation, characterization, another policy, or a second bridge
    at the same time. More than one publisher on `/drone_attitude_cmd` or
    `/drone_actuator_cmd` makes the result unsafe and impossible to interpret.

For a cautious two-step start, launch with `vq1_controller_enabled:=false`, verify telemetry, then enable the controller:

```bash
export ROS_LOCALHOST_ONLY=1

ros2 topic echo --once /aigp/armed
ros2 topic echo --once /aigp/ground_truth/state
ros2 topic echo --once /aigp/active_gate_index

ros2 service call /aigp_vq1_controller/enable \
  std_srvs/srv/SetBool "{data: true}"
```

---

## Architecture and communication

The simulator normally runs on Windows while ROS 2 runs in WSL2 or the development container. MAVLink telemetry and commands use UDP `14550`; chunked JPEG camera data uses UDP `5600`.

```mermaid
flowchart LR
    SIM[FlightSim VQ1/VQ2]:::m-base
    BRIDGE[AIGP MAVLink bridge]:::m-dep
    GT[VQ1 ground-truth controller]:::m-target
    CAMERA[Camera driver]:::m-dep
    DETECT[Gate detector]:::m-dep
    ESKF[VQ2 IMU + vision ESKF]:::m-target
    POLICY[VQ2 policy]:::m-target
    MOTORS[Direct motor actuators]:::m-final

    SIM -- "MAVLink :14550<br/>ODOMETRY, IMU, race status" --> BRIDGE
    BRIDGE -- "/aigp/ground_truth/state<br/>VQ1 only" --> GT
    GT -- "/drone_attitude_cmd" --> BRIDGE

    SIM -- "JPEG chunks :5600" --> CAMERA
    CAMERA -- "/cam0/image_rgb" --> DETECT
    BRIDGE -- "/imu0" --> ESKF
    DETECT -- "gate pose correction" --> ESKF
    ESKF -- "/drone_state_estimated" --> POLICY
    POLICY -- "/drone_attitude_cmd" --> BRIDGE

    BRIDGE -- "SET_ACTUATOR_CONTROL_TARGET" --> MOTORS
    MOTORS --> SIM

    classDef m-base fill:var(--mermaid-base-bg),stroke:var(--mermaid-base-border),color:var(--mermaid-base-fg)
    classDef m-dep fill:var(--mermaid-dep-bg),stroke:var(--mermaid-dep-border),color:var(--mermaid-dep-fg)
    classDef m-target fill:var(--mermaid-target-bg),stroke:var(--mermaid-target-border),color:var(--mermaid-target-fg)
    classDef m-final fill:var(--mermaid-final-bg),stroke:var(--mermaid-final-border),color:var(--mermaid-final-fg),stroke-width:2px
```

VQ1 and VQ2 share the bridge and actuator path but use different state sources. VQ1 bypasses the camera and estimator. VQ2 must never consume `/aigp/ground_truth/state`.

---

## Components

### 1. `aigp_bridge_node`
A dedicated Python ROS2 node that replaces the MAVSDK client. It handles:
- **Telemetry Parsing:** Republishes `HIGHRES_IMU` as `/imu0`. VQ1 build 3390 also provides `ODOMETRY`, `ATTITUDE`, and `LOCAL_POSITION_NED`.
- **VQ1 Ground Truth:** Publishes a coherent 13D state from `ODOMETRY` on `/aigp/ground_truth/state` (and the legacy `/drone_state_sim` alias). Body-frame odometry velocity is rotated into world NED. Do not combine its quaternion with the simulator's `ATTITUDE` packet, whose pitch convention differs in build 3390.
- **Clock Synchronization:** Sends 10Hz `TIMESYNC` requests to keep simulator and ROS clocks aligned.
- **Control Translation:** Converts `/drone_attitude_cmd` (deg/s) into four motor commands using HIGHRES_IMU rate feedback. The old `SET_ATTITUDE_TARGET` path remains an explicit fallback, but its rate fields produced no live response in build 3390.
- **Direct Actuator Control:** Converts `/drone_actuator_cmd` motor arrays into MAVLink `SET_ACTUATOR_CONTROL_TARGET` controls.
- **Velocity-Control Probe:** Converts `/drone_velocity_cmd` (`[vx, vy, vz, yaw_rate]`) into body- or local-NED `SET_POSITION_TARGET_LOCAL_NED` commands for interface characterization.
- **Direct IMU Stream:** Optionally republishes `HIGHRES_IMU` directly as `/imu0` (`sensor_msgs/msg/Imu`) to prevent port conflicts with C++ adapters.
- **Consistent Timestamps:** Uses local ROS receipt time by default. `HIGHRES_IMU.time_usec` is boot-relative while camera `sim_time_ns` is server-epoch time, so raw stamps are not directly comparable.
- **Diagnostics Feedback:** Publishes collision and applied-actuator telemetry; optional `/aigp/highres_imu_aux` exposes magnetometer/pressure fields for availability checks.
- **Race Status:** Parses AIGP encapsulated race-status packets and publishes `/aigp/active_gate_index` plus `/aigp/race_status`.

### 2. `aigp_vq1_controller_node`

This is the completion-first controller selected by `controller_type:=aigp_vq1_gt`. It does not assume a known mass, inertia tensor, or motor model. Instead, it combines VQ1 ground truth with parameters measured directly from the running simulator.

At each update it:

1. Reads the active gate index reported by FlightSim.
2. Commands a constant, low velocity through the gate plane.
3. Independently centers the drone in the two directions perpendicular to the gate.
4. Converts the velocity error into bounded acceleration and attitude targets.
5. Uses the bridge's IMU-feedback actuator backend to track body rates.
6. Cuts thrust on race completion or a high-severity safety fault.

```mermaid
flowchart LR
    STATE[Ground-truth state]:::m-base
    GATE[Active gate + static track]:::m-base
    FIELD[Gate velocity field]:::m-dep
    OUTER[Velocity and attitude loop]:::m-target
    RATE[IMU body-rate loop]:::m-target
    MIX[Four-motor mixer]:::m-final

    STATE --> FIELD
    GATE --> FIELD
    FIELD --> OUTER
    STATE --> OUTER
    OUTER --> RATE
    STATE --> RATE
    RATE --> MIX

    classDef m-base fill:var(--mermaid-base-bg),stroke:var(--mermaid-base-border),color:var(--mermaid-base-fg)
    classDef m-dep fill:var(--mermaid-dep-bg),stroke:var(--mermaid-dep-border),color:var(--mermaid-dep-fg)
    classDef m-target fill:var(--mermaid-target-bg),stroke:var(--mermaid-target-border),color:var(--mermaid-target-fg)
    classDef m-final fill:var(--mermaid-final-bg),stroke:var(--mermaid-final-border),color:var(--mermaid-final-fg),stroke-width:2px
```

#### Important simulator conventions

| Item | VQ1 convention used by the controller |
| --- | --- |
| World frame | NED in telemetry; converted to NWU for track logic |
| Body frame | FRD in telemetry; converted to FLU where required |
| Linear velocity | `ODOMETRY` body velocity rotated into world NED by the bridge |
| Attitude source | `ODOMETRY` quaternion, not the inconsistent `ATTITUDE` packet |
| Roll mapping | Reflected target geometry, normal measured-rate feedback |
| Pitch mapping | Simulator-specific inverted rate sign |
| Gate height | Track Z is gate-base elevation; add `1.35 m` to target the center of a `2.7 m` gate |
| Final gate index | Index `6` means all six gates passed; it is completion, not an invalid index |

The separate vertical gains are intentional. Early tests held lateral position well but flew about `0.18 m` too low and touched the lower gate frame. Stronger gains on every axis caused lateral oscillation. Keeping gentle lateral gains while giving altitude its own centering authority produced repeatable `0.01–0.05 m` gate-center errors.

!!! info "Relationship to the Isaac result"
    The high-level gate velocity field comes from the ground-truth controller that
    completed the AIGP track in Isaac. The live FlightSim controller is not the PPO
    network itself: it is the deterministic completion-first portion, paired with
    live actuator and thrust calibration. This distinction matters when reporting
    results or planning VQ2 work.

### 3. `aigp_state_estimator_node`
VQ2 does not permit ground truth. With `state_source:=estimator`, this `rl` node runs a 15-error-state ESKF using high-rate IMU propagation and visual gate pose corrections from `gate_detector_node`. VQ1 instead selects the bridge's ground-truth state and does not start the estimator or gate detector.

It outputs a 13-value NED/FRD state vector on `/drone_state_estimated`:

```text
[position_ned(3), velocity_ned(3), quaternion_wxyz(4), angular_velocity_frd(3)]
```

Key fusion features:
- **Bias-Aware Propagation:** Estimates accelerometer and gyroscope biases with a full 15×15 covariance.
- **Stationary Initialization:** Uses the pre-start IMU window for roll/pitch and bias initialization.
- **Known Start Heading:** Seeds yaw to pi radians from the fixed competition start pose; gravity alone cannot observe yaw.
- **Statistical Vision Fusion:** Consumes the gate detector's 6×6 covariance and rejects inconsistent updates with a chi-square innovation test.
- **Delayed Measurements:** Rewinds up to 0.5 s to the camera timestamp and replays buffered IMU samples.
- **Filter Health:** Publishes acceptance, NIS, rewind age, covariance scales, and bias norms on `/aigp_state_estimator/diagnostics`.

### 4. `gate_detector_node`
A dedicated vision node inside package `gate_detection` that processes camera frames from `/cam0/image_rgb`:
- **Neon Orange Color Thresholding**: Dual-range HSV segmentation (`H: 0..25` and `170..180`, `S: 90..255`, `V: 70..255`) tailored for neon orange AIGP race gates.
- **Adaptive Corner Extraction**: Multi-stage corner solver (`extract_gate_corners`) with polygon approximation sweeps, convex hull quadrant vertex extraction, and minimum area rectangle fallbacks.
- **Candidate Association:** Scores gate-like rings using quadrilateral geometry and temporal continuity rather than selecting the largest orange blob.
- **Validated Planar Pose:** Resolves IPPE ambiguity and rejects invalid depth, distance, or reprojection error before publishing a quality-derived covariance.
- **Inertially Constrained Distant Gates:** Uses the estimator attitude to resolve planar PnP orientation ambiguity, then requires at most 3 m position innovation and 2 px reprojection error. Measurements fail closed when the camera and inertial state are inconsistent.
- **Full Camera Extrinsic:** Converts optical `x-right, y-down, z-forward` coordinates to body FLU. Although the nominal mount is documented as 20 degrees upward, VQ1 ground-truth projection calibrates build 3390's effective optical tilt to 17.3 degrees; using 20 degrees caused about 1.1 m of vertical pose bias at the start gate.

### 5. `VisionDriverNode`
A C++ component inside `aigp_camera_driver` that listens on port `5600`, reassembles chunked-JPEG UDP packets, decodes them using OpenCV, and publishes standard ROS image topics (`/cam0/image_raw` mono8, `/cam0/image_rgb` rgb8).

### 6. `aigp_policy_node`
A dedicated `rl` node that inspects the TorchScript input contract. It supports Nicola's legacy 25D GRU and the exact 105D, three-sample-history `ours` observation used by the new AIGP sensor-policy task.

!!! warning
    The old 105D tamed checkpoint now exports correctly, but it was trained under
    mismatched dynamics, action, and track settings. Do not treat it as
    flight-validated. Retrain with `aigp_sensor_policy.yaml` and complete
    closed-loop AIGP validation.

---

## Running the simulator

### Step 1: start FlightSim

Launch `FlightSim.exe` on the host machine. Confirm that the window reports the VQ1 build when ground truth is required. FlightSim starts sending UDP streams to `127.0.0.1`.

### Step 2: start the development environment
By default, the container starts with CPU/software rendering (which runs on all systems). Build and run the `aigp` compose service:

```bash
# Build the container
docker compose build dev

# Start the container
docker compose up aigp -d

# Exec into the container
docker compose exec aigp bash
```

??? tip "Optional host GPU acceleration"
    For Intel or AMD, mount the direct-rendering device in `docker-compose.yaml`:

    ```yaml
    devices:
      - /dev/dri:/dev/dri
    ```

    For NVIDIA, configure the NVIDIA Container Toolkit runtime:

    ```yaml
    environment:
      - NVIDIA_VISIBLE_DEVICES=all
      - NVIDIA_DRIVER_CAPABILITIES=all
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: all
              capabilities: [gpu]
    ```

### Step 3: build and select a mode

Inside the ROS 2 workspace:

```bash
colcon build --packages-select rl betaflight_mavlink_bridge drone_control gate_detection aigp_camera_driver aigp_imu_driver
source install/setup.bash
export ROS_LOCALHOST_ONLY=1

# Validated VQ1 completion controller
ros2 launch drone_control aigp_bringup.launch.py \
  controller_type:=aigp_vq1_gt \
  vq1_controller_enabled:=true \
  rate_control_backend:=actuator

# VQ1 policy experiment (not the validated completion controller)
ros2 launch drone_control aigp_bringup.launch.py \
  controller_type:=aigp_policy state_source:=ground_truth

# VQ2: the same policy with IMU + gate-vision state estimation
ros2 launch drone_control aigp_bringup.launch.py \
  controller_type:=aigp_policy state_source:=estimator

# Nicola RNN racing policy bringup with real-time graphical debug dashboard
ros2 launch drone_control aigp_bringup.launch.py controller_type:=aigp_policy debug:=true

# Interactive keyboard teleoperation with real-time graphical state estimator dashboard
ros2 launch drone_control aigp_bringup.launch.py controller_type:=teleop
```

The `aigp_vq1_gt` mode starts the bridge and VQ1 controller without the camera or estimator. The `aigp_policy` mode starts the bridge, camera, and policy. With `state_source:=estimator`, it additionally starts the gate detector and ESKF. Passing `debug:=true` or `controller_type:=teleop` launches `estimator_visualizer_node` with the selected state topic.

Useful checks:

```bash
export ROS_LOCALHOST_ONLY=1
export ROS_NO_DAEMON=1

ros2 topic hz /aigp/active_gate_index
ros2 topic hz /imu0
ros2 topic hz /aigp/ground_truth/state  # VQ1
ros2 topic hz /drone_state_estimated    # VQ2
ros2 topic hz /drone_attitude_cmd
ros2 topic echo /aigp/race_status --once
ros2 topic echo /aigp_state_estimator/diagnostics
ros2 service call /aigp_state_estimator/reset std_srvs/srv/Trigger
ros2 service call /aigp_policy/reset std_srvs/srv/Trigger
```

Both controller paths use the packaged static competition track and live race-status packets for the active gate. Estimator linear velocity is world NED, so the default `velocity_frame:=world_ned` must remain in use for VQ2.

### VQ1 controller services

The completion controller starts disabled unless explicitly enabled through the launch argument. These services allow a team member to validate telemetry before flight or recover without restarting every ROS process:

```bash
# Enable autonomous control
ros2 service call /aigp_vq1_controller/enable \
  std_srvs/srv/SetBool "{data: true}"

# Stop commands and cut thrust
ros2 service call /aigp_vq1_controller/enable \
  std_srvs/srv/SetBool "{data: false}"

# Clear controller state after resetting FlightSim
ros2 service call /aigp_vq1_controller/reset \
  std_srvs/srv/Trigger "{}"

# Reset FlightSim itself
ros2 service call /reset_sim std_srvs/srv/Trigger "{}"
```

---

## VQ1 validation record

The successful run was performed against FlightSim `1.0.3390-VQ1` on 24 July 2026. The run advanced through gate indices `0 → 1 → 2 → 3 → 4 → 5 → 6`, then emitted the race-finish event. Approximate wall-clock duration from enable to finish was `165 s`.

| Setting | Validated value | Why it is conservative |
| --- | ---: | --- |
| Forward target speed | `1.0 m/s` | Leaves ample time to center before every gate |
| Lateral position gain | `0.3 s⁻¹` | Avoids the oscillation seen with aggressive centering |
| Lateral speed limit | `0.5 m/s` | Bounds gate-to-gate corrections |
| Vertical position gain | `1.0 s⁻¹` | Removes the low-altitude gate-frame contact |
| Vertical speed limit | `0.5 m/s` | Handles the steep course descent without a large transient |
| Acceleration limit | `0.5 m/s²` | Keeps attitude demands small |
| Body-rate limit | `0.5 rad/s` | Keeps the unknown vehicle comfortably inside the validated region |
| Gate-center height offset | `1.35 m` | Half the physical `2.7 m` gate height |
| Corridor safety limit | `20 m` | Accepts legitimate target jumps; the largest is about `12.6 m` |

### Why the first approaches failed

Understanding the failed approaches prevents the next team member from repeating the same tests.

| Attempt | What happened | Resolution |
| --- | --- | --- |
| `SET_POSITION_TARGET_LOCAL_NED` velocity control | Even very small commands saturated the motors and caused vertical runaway | Do not use the simulator velocity target for VQ1 control |
| V4 `SET_ATTITUDE_TARGET` body rates | Response was delayed or inconsistent enough to destabilize the outer loop | Use the bridge's local IMU-feedback actuator backend |
| One roll/pitch convention for every interface | The vehicle accelerated toward the gate but could not brake laterally | Calibrate axes independently; use the VQ1 mappings in the controller runtime |
| Weak vertical centering | The drone stabilized about `0.18 m` too low and hit the lower gate frame | Use separate, stronger vertical gains |
| Treating track Z as aperture center | The controller aimed at gate-base elevation | Add the `1.35 m` center-height offset |
| `8 m` centering safety corridor | Normal gate transitions of `9–13 m` triggered a reset | Use the validated `20 m` guard |
| Stopping on every threat-1 collision packet | A valid gate-3 transition was reset even though the race advanced | Stop only on simulator threat level 2; threat level 1 also occurs on valid transitions |
| Treating active gate index `6` as invalid | The completed lap briefly requested a reset before the finish packet | Index `6` is the terminal state for a six-gate track |

!!! note "What remains unknown"
    We did not infer a mass, inertia tensor, propeller model, or complete aerodynamic
    model for the simulated vehicle. The VQ1 controller does not need them. Its
    thrust map and rate loop were measured from the running simulator and should be
    rechecked after any FlightSim vehicle or physics update.

## Arming and controlling the simulator

Unlike physical Betaflight deployments using MSP, the AIGP simulator uses MAVLink commands for control:

### Keyboard teleoperation and debug dashboard

You can fly the drone manually using keyboard inputs while observing state estimator performance in real time via the graphical debug dashboard:

```bash
# Launch interactive keyboard teleop + real-time graphical debug dashboard
ros2 launch drone_control aigp_bringup.launch.py controller_type:=teleop
```

Key mappings for keyboard flight control:

| Key | Control Action | Command Range |
|---|---|---|
| `w` / `s` | **Pitch Forward / Backward** rate | $\pm 45^\circ/\text{s}$ |
| `a` / `d` | **Roll Left / Right** rate | $\pm 45^\circ/\text{s}$ |
| `q` / `e` | **Yaw Left / Right** rate | $\pm 60^\circ/\text{s}$ |
| `i` / `k` | **Thrust Up / Down** | $0.0 \dots 1.0$ (build-3390 level hover was about `0.265`) |
| `SPACE` | **Arm / Disarm** drone | Calls `/arm_drone` service |
| `r` | **Reset Simulation** | Calls `/reset_sim` service |
| `x` | **Emergency Zero Rates** | Zeroes all rates & sets hover thrust |

### Real-time graphical debug dashboard (`debug:=true`)

To monitor state estimation, visual gate updates, and trajectory tracking graphically under any controller (including Nicola's RL policy), pass `debug:=true`:

```bash
ros2 launch drone_control aigp_bringup.launch.py controller_type:=aigp_policy debug:=true
```

The graphical dashboard (`estimator_visualizer_node`) renders a live window and publishes `/estimator_viz/image_raw` containing:

- **2D Top-Down Flight Trajectory Map (NED)**: Real-time green flight path trail, blue track gates ($G_0 \dots G_5$), and orange markers representing visual SolvePnP gate pose corrections.
- **Onboard Camera HUD**: Live `/cam0/image_rgb` video stream with an **orange flash indicator** `[PNP GATE CORRECTION FUSED]` whenever visual gate pose updates refine the state estimator.
- **Telemetry Gauges**: Real-time position ($X, Y, Z$), linear velocity ($V_x, V_y, V_z$), speed ($\text{m/s}$), RPY tilt angles, body rotation rates, active target gate, and total PnP pose resets.

!!! note "Headless autonomous mode"
    `is_auto` defaults to `True` in `aigp_bridge_node`. Autonomous runs therefore
    do not require a gamepad or joystick.

---

## Drone characterization

The stack includes a black-box characterization tool for estimating AIGP simulator parameters from commanded actuator inputs and telemetry. Use this before controller tuning when the simulator dynamics or race environment has changed.

The test estimates:

- hover actuator command
- effective thrust-to-weight ratio
- vertical thrust command fit
- thrust step response
- roll, pitch, and yaw actuator-to-rate response
- stationary IMU noise statistics, when `/imu0` is available

Run the AIGP bridge without another controller publishing to `/drone_attitude_cmd` or `/drone_actuator_cmd`, then start the test node.
By default, the test sends direct actuator commands on `/drone_actuator_cmd`, because AIGP responds reliably to MAVLink `SET_ACTUATOR_CONTROL_TARGET`.

### Why actuator commands

Live build-3390 tests showed that AIGP accepts collective thrust through `SET_ATTITUDE_TARGET`, but its body-rate response was absent in some tests and delayed or inconsistent in others, even with the V4 bit-16 physical-rate extension. It was not suitable for the outer flight-control loop. Direct actuator output was reliable, so the bridge closes the body-rate loop locally with HIGHRES_IMU feedback and sends `SET_ACTUATOR_CONTROL_TARGET` by default (`rate_control_backend:=actuator`).

The bridge maps:

| ROS topic | Message | MAVLink output |
| --- | --- | --- |
| `/drone_attitude_cmd` | `Float64MultiArray [roll_deg_s, pitch_deg_s, yaw_deg_s, thrust]` | local rate controller, then `SET_ACTUATOR_CONTROL_TARGET` |
| `/drone_actuator_cmd` | `Float64MultiArray [m0, m1, m2, m3, ...]` | `SET_ACTUATOR_CONTROL_TARGET` |

Only the first four actuator entries are used for motor testing. Values are normalized in `[0.0, 1.0]`.

### Safety timing

AIGP can disqualify the run for early start. The characterization tool therefore waits at zero command after every reset before sending nonzero actuator values.

Keep:

```text
start_delay_s >= 4.0
```

The live simulator reported an early-start disqualification at `2700 ms`, so `4.0 s` gives a margin.

### Motor patterns

The actuator tests use equal motor commands for collective thrust:

```text
[u, u, u, u]
```

For angular response tests, the empirically verified AIGP motor patterns are:

| Target axis | Motor command pattern around hover `h` |
| --- | --- |
| Roll | `[h - d, h + d, h - d, h + d]` |
| Pitch | `[h - d, h - d, h + d, h + d]` |
| Yaw | `[h + d, h - d, h - d, h + d]` |

The default differential step is:

```text
actuator_step_delta = 0.08
```

For stronger identification, `0.12` produced clear live angular response, but use it carefully because the drone can move aggressively.

### Running the test

```bash
colcon build --packages-select drone_control betaflight_mavlink_bridge
source install/setup.bash

# Terminal 1: bridge only
ros2 run betaflight_mavlink_bridge aigp_bridge_node

# Terminal 2: scripted characterization
ros2 run drone_control aigp_characterization
```

The node writes a raw CSV and fitted JSON metrics under `aigp_characterization_runs/`.

Recommended explicit command:

```bash
ros2 run drone_control aigp_characterization --ros-args \
  -p output_dir:=/tmp/aigp_characterization \
  -p command_mode:=actuator \
  -p start_delay_s:=4.0 \
  -p takeoff_thrust:=0.75 \
  -p thrust_test_duration_s:=1.2 \
  -p actuator_step_delta:=0.08
```

For a rate-only run with a known collective baseline, skip the vertical fit:

```bash
ros2 run drone_control aigp_characterization --ros-args \
  -p run_thrust_sweep:=false \
  -p run_thrust_step_tests:=false \
  -p run_rate_tests:=true \
  -p rate_test_thrust_override:=0.5
```

The thrust sweep intentionally performs a short high-thrust pop before each test point so the acceleration fit is less affected by ground contact. If the resulting JSON reports too few airborne thrust points, increase `takeoff_thrust` or `takeoff_duration_s` slightly.

### Outputs

Each run writes:

```text
aigp_characterization_<timestamp>.csv
aigp_characterization_<timestamp>.json
```

The CSV contains command, state, and IMU samples for plotting. The JSON contains fitted metrics:

| JSON field | Meaning |
| --- | --- |
| `hover_thrust` | Normalized collective actuator command where vertical acceleration crosses zero |
| `thrust_to_weight` | Effective `T_max / weight` estimate |
| `thrust_fit` | Linear fit from command to vertical acceleration |
| `body_rate_response` | Angular-rate response to differential actuator steps |
| `imu_noise` | Static IMU mean/stddev, if enough IMU samples were recorded |

The vertical fit uses the AIGP/NED convention where positive `z` is down. With a linear fit:

```text
az = slope * u + intercept
hover = -intercept / slope
TWR = (g - az_at_u_1) / g
```

### Live measurements

The VQ1 completion work measured the following values on FlightSim `1.0.3390-VQ1`:

| Measurement | Value |
| --- | ---: |
| Affine vertical fit | `az_NED = 8.420 - 30.336 u` |
| Fit RMSE | `0.380 m/s²` |
| Hover command in the pitched calibration pose | `0.278` |
| Level-flight collective seen during the completed lap | approximately `0.265` |
| Nominal thrust-to-weight estimate | `3.23` |
| Reset-pose tilt | approximately `17.8°` |

The difference between pitched-pose hover and level-flight collective is expected: the reset vehicle starts tilted, so less of its thrust acts vertically. The controller accounts for current body-up projection rather than using one fixed number everywhere.

!!! warning "Do not copy old thrust values"
    Thrust values are simulator-build and backend dependent. Re-run characterization
    after a simulator update, and never assume the physical 10-inch drone has the
    same normalized hover command.

A direct live MAVLink actuator test was run with:

```text
start_delay_s = 4.1
actuator_step_delta = 0.12
```

The following collective and open-loop step values came from an earlier simulator revision and must not be treated as build-3390 vehicle parameters:

| Metric | Value |
| --- | --- |
| Hover command | `0.418` |
| Effective thrust-to-weight | `2.33` |
| Vertical fit | `az = -22.42 * u + 9.38` |
| Fit RMSE | `1.27 m/s^2` |
| Roll max rate / tau | `8.78 rad/s`, `0.132 s` |
| Pitch max rate / tau | `8.77 rad/s`, `0.130 s` |
| Yaw max rate / tau | `6.32 rad/s`, `0.193 s` |

On build 3390, a separate closed-loop check at a `1.0 rad/s` target measured mean final rates of `0.997`, `0.992`, and `1.004 rad/s` for roll, pitch, and yaw respectively with the bridge's default actuator gains. This validates rate tracking, not mass, inertia, maximum thrust, or a 10-inch vehicle model. Those physical parameters remain unverified and must not be inferred from the older table.

### Troubleshooting characterization

If the ROS graph hangs or characterization reports no `/imu0`, first confirm the simulator is sending MAVLink:

```bash
ros2 run betaflight_mavlink_bridge aigp_bridge_node
```

Healthy current-simulator logs must show increasing `HIGHRES_IMU` counts. VQ1 build 3390 also emits `ODOMETRY`, `LOCAL_POSITION_NED`, and `ATTITUDE`; VQ2 must continue to work without them. Characterization records and fits IMU-only runs, so ground truth is not required for rate identification.

If MAVLink is healthy but ROS topic discovery fails, check the WSL2/DDS notes below. In our host-side tests, `rmw_fastrtps_cpp` discovery between local Python processes was unreliable, while raw MAVLink telemetry was healthy. Prefer running inside the configured Docker service with the workspace environment sourced.

---

## Troubleshooting

### ROS 2 cannot see local topics or services

WSL2 discovery is most reliable when every terminal uses the same loopback setting:

```bash
source install/setup.bash
export ROS_LOCALHOST_ONLY=1
export ROS_NO_DAEMON=1

ros2 topic list
ros2 topic hz /aigp/ground_truth/state
```

Do not append `--no-daemon` to `ros2 topic hz`; that verb does not accept the option in the ROS 2 version used by this project. The `ROS_NO_DAEMON=1` environment variable works consistently across topic verbs. The Docker service also provides a CycloneDDS loopback configuration through `src/cyclonedds.xml`.

### Bridge waits forever for a heartbeat

- Confirm FlightSim is running before the bridge.
- Confirm no other process already owns UDP port `14550`.
- Stop any old `aigp_bridge_node` before starting a new bringup.
- Check that Windows/WSL networking still forwards the simulator UDP stream.

### Ground truth is missing

Ground truth is only available in VQ1. A healthy VQ1 bridge publishes a 13-value state:

```bash
ros2 topic echo --once /aigp/ground_truth/state
```

If IMU data exists but this topic does not, verify the FlightSim build and keep `ground_truth_source:=odometry`. Do not reconstruct the VQ1 quaternion from `ATTITUDE`; build 3390 uses a different pitch convention there.

### Drone runs away or motors saturate

1. Stop the controller with `/aigp_vq1_controller/enable` set to `false`.
2. Reset FlightSim with `/reset_sim`.
3. Confirm `rate_control_backend:=actuator`.
4. Confirm there is only one `/drone_attitude_cmd` publisher.
5. Restore the documented VQ1 defaults before tuning anything.

The stabilized velocity-target interface is not a fallback: live VQ1 tests produced full motor saturation and vertical runaway from very small velocity commands.

### Gate index advances but a collision packet appears

FlightSim can publish threat-level-1 collision packets during a valid transition. If `/aigp/active_gate_index` advances, the gate passed. The controller stops only on threat level 2. A gate index equal to the gate count (`6` on this track) is the terminal completion state.

### Reproducing the validation checks

```bash
source install/setup.bash

PYTHONPATH=src/rl:src/utils/betaflight_mavlink_bridge:src/drone_control:${PYTHONPATH:-} \
pytest-3 -q \
  src/rl/test/test_aigp_vq1_controller_runtime.py \
  src/rl/test/test_aigp_policy_runtime.py \
  src/utils/betaflight_mavlink_bridge/test/test_aigp_bridge_control.py \
  src/drone_control/test/test_aigp_characterization.py
```

At the time of the completed lap this suite reported **22 passed, 2 skipped**.
