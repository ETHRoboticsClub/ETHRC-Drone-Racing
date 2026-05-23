# EKF Setup Guide — Quaternion DroneModel Integration

> **Audience**: Team members integrating the full perception pipeline (VIO + Vision + EKF + Controls).
> If you are running the pipeline for the first time without the EKF author present, **read this entire document**.

---

## What Changed (and Why It Matters to You)

We replaced `robot_localization`'s stock 15-state Euler EKF with a **16-state quaternion EKF** that uses a **drone physics model** in the predict step. The filter lives in a modified fork of `robot_localization` (submodule at `src/perception/ekf/robot_localization/`).

**Everything downstream is unchanged**: same topic names, same `nav_msgs/Odometry` output on `/ekf/odometry`, same message types. The output orientation is now computed from quaternion state internally, so it is free of gimbal-lock singularities.

### Why we changed it

The stock Euler EKF had two problems critical for drone racing:

1. **Gimbal lock**: The Euler rate matrix has a `1/cos(pitch)` term that blows up at +-90 pitch. Aggressive gate dives and pull-ups hit this.
2. **Wrong physics**: The stock filter uses a constant-velocity model — no quaternion coupling, no gravity-induced acceleration, no angular rate decay. Our replacement uses first-principles quadrotor dynamics.

---

## 1. The State Vector

```
Index:  0   1   2   3    4    5    6    7   8   9   10      11       12     13  14  15
State: [X,  Y,  Z,  Qw,  Qx,  Qy,  Qz,  Vx, Vy, Vz, Vroll, Vpitch, Vyaw,  Ax, Ay, Az]
        ─────────── ──────────────────── ─────────── ───────────────── ───────────────
        World pos    Quaternion (body→    Body-frame   Body angular     Body-frame
        (ENU)        world, unit-norm)    velocity     rates (rad/s)    acceleration
```

**Key difference**: Orientation was 3 values (Roll/Pitch/Yaw at indices 3-5). Now it is 4 values (Qw/Qx/Qy/Qz at indices 3-6). Every index from Vx onward shifted by +1.

### Config arrays are now 16 booleans

All `_config` parameters in YAML must have **16 booleans** (was 15):

```yaml
# [x, y, z, qw, qx, qy, qz, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az]
odom0_config: [true, true, true,   true, true, true, true,   true, true, true,
               true, true, true,  false, false, false]
```

> **WARNING**: Writing 15 booleans will silently truncate and map sensor data to wrong state indices. Always count to 16.

---

## 2. The DroneModel Predict Step

The predict step (`ekf.cpp::predictDt()`) propagates the state forward using quadrotor dynamics. This is the core change and the reason the filter performs differently from stock.

### Quaternion kinematics
```
q_new = q + 0.5 * Xi(q) * omega_body * dt     (then re-normalized)
```
Xi(q) is the 4x3 quaternion rate matrix. Propagates orientation from angular rates (Vroll, Vpitch, Vyaw).

### Position kinematics
```
p_new = p + R_wb * v_body * dt + 0.5 * R_wb * a_body * dt^2
```
R_wb is the body-to-world rotation matrix from the quaternion. **Output twist is in body frame.**

### Gravity coupling (how attitude affects acceleration)
```
Ax = -2g * (Qw*Qy + Qx*Qz)     (zero at hover, grows with tilt)
Ay = -2g * (Qy*Qz - Qw*Qx)
Az = 0                            (hover assumption: thrust ~ mg)
```

### Angular rate decay
```
omega_new = omega * (1 - dt/tau_w)     tau_w = 0.1 s
```
First-order model: rates decay toward zero when no new command arrives. This allows the EKF to dead-reckon orientation during short sensor dropouts.

### Reference implementation
The C++ predict step (`src/perception/ekf/robot_localization/src/ekf.cpp`) is a direct translation of the Python model at `src/mpc/mpc/drone_model.py`.

---

## 3. Building

### Full workspace build (first time)

```bash
# Inside Docker container:
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

`robot_localization` is built from our modified submodule, not the system apt package.

### Rebuilding after changes

```bash
# EKF package only (launch files, configs, mock publisher):
colcon build --symlink-install --packages-select ekf --cmake-args -DBUILD_TESTING=OFF

# If you also changed robot_localization source:
colcon build --symlink-install --packages-select robot_localization
colcon build --symlink-install --packages-select ekf --cmake-args -DBUILD_TESTING=OFF
```

> **`robot_localization` is large** (heavy Eigen templates). First build takes 2-5 minutes. Subsequent builds are fast unless you changed its source files. Do not `rm -rf build/robot_localization` unless necessary.

### Build error: "symbolic link ... is a directory"

```bash
rm -rf build/robot_localization/ament_cmake_python/robot_localization/robot_localization
colcon build --symlink-install --packages-select robot_localization --allow-overriding robot_localization
```

---

## 4. Running the Full Pipeline

### Step 1: Start PX4 SITL

```bash
cd /home/drone_dev/ros_ws/px4/PX4-Autopilot
make px4_sitl gz_my_drone
```

### Step 2: Launch the perception pipeline

```bash
source install/setup.bash
ros2 launch launch/perception.launch.py
```

This starts:
1. **gazebo_ros_bridges** — Gazebo camera + IMU to ROS 2
2. **aruco_pose_estimation** — ArUco markers, publishes `/aruco/drone_pose` (`PoseWithCovarianceStamped`)
3. **ov_msckf** (OpenVINS) — VIO, publishes `/ov_msckf/odomimu` (`Odometry`)
4. **ekf_filter_node** — Quaternion EKF, publishes `/ekf/odometry` (`Odometry`)

### Step 3: Sanity check

```bash
# EKF is publishing at ~100 Hz:
ros2 topic hz /ekf/odometry

# Both sensors are feeding the EKF:
ros2 topic hz /ov_msckf/odomimu
ros2 topic hz /aruco/drone_pose

# Orientation is not zero (filter has initialized):
ros2 topic echo /ekf/odometry --field pose.pose.orientation --once
# w should be close to 1.0 at hover, x/y/z close to 0.0
# If all four are 0.0, the filter has not received its first measurement yet — wait for VIO.

# Quaternion is unit-length:
# w^2 + x^2 + y^2 + z^2 should be ~ 1.0
```

### Step 4: Launch the controller

```bash
source install/setup.bash
ros2 launch drone_control drone_controller.launch.py
```

---

## 5. Configuration

The production config is `config/rl_ekf.yaml`.

### Sensor inputs

| Parameter | Topic | Message type | What it fuses |
|-----------|-------|-------------|---------------|
| `odom0` | `/ov_msckf/odomimu` | `nav_msgs/Odometry` | Position, orientation, velocity, angular rates |
| `pose0` | `/aruco/drone_pose` | `geometry_msgs/PoseWithCovarianceStamped` | Absolute position + orientation |

### Default sensor configs

```yaml
# VIO — fuses everything except acceleration:
odom0_config: [true, true, true,   true, true, true, true,   true, true, true,
               true, true, true,  false, false, false]

# Vision — only absolute pose:
pose0_config: [true, true, true,   true, true, true, true,
               false, false, false, false, false, false, false, false, false]
```

### Important parameters

| Parameter | Default | Notes |
|-----------|---------|-------|
| `frequency` | 100.0 | EKF output rate (Hz). Must match or exceed fastest sensor. |
| `sensor_timeout` | 0.1 | Seconds without measurement before filter just predicts. |
| `predict_to_current_time` | true | Extrapolates state for low-latency output. |
| `smooth_lagged_data` | true | Handles out-of-order measurements. |
| `history_length` | 0.5 | Seconds of state history for smoothing. If your vision latency exceeds this, increase it or measurements get silently dropped. |
| `use_control` | false | Set `true` if `/control/command` (`TwistStamped`) is available. Improves prediction during sensor gaps. |
| `publish_tf` | true | Publishes `odom -> base_link` transform. |
| `publish_acceleration` | true | Publishes `/accel/filtered`. |

### Coordinate frames

```
world (ENU)
  └── odom
       └── base_link
            └── camera_link
```

The EKF operates in **ENU** (East-North-Up). PX4 uses NED internally; the MAVSDK bridge handles conversion. **Do not convert frames between VIO/Vision and EKF** — they already publish in ENU-compatible frames.

---

## 6. Startup Behavior

On startup the filter state is all zeros, including the quaternion `[Qw=0, Qx=0, Qy=0, Qz=0]`. The filter is **not initialized** until the first measurement arrives.

**What happens before initialization**:
- The EKF publishes zero orientation (not identity!)
- The first measurement (usually VIO) sets the initial state
- VIO (OpenVINS) typically takes 2-5 seconds to initialize after camera data starts flowing

**Implication for controls**: The drone controller should **not trust EKF output until the orientation is non-zero**. A simple check: `w^2 + x^2 + y^2 + z^2 > 0.5` means the filter is initialized. Before that, do not send position commands.

---

## 7. Output Format

The EKF publishes on `/ekf/odometry` (`nav_msgs/Odometry`):

| Field | Frame | Source |
|-------|-------|--------|
| `pose.pose.position` | World (ENU) | State indices 0-2 |
| `pose.pose.orientation` | Body→world quaternion | State indices 3-6 (unit normalized) |
| `twist.twist.linear` | **Body frame** | State indices 7-9 |
| `twist.twist.angular` | **Body frame** | State indices 10-12 |
| `pose.covariance` | 6x6 (pos, RPY) | Position block exact; orientation block projected from 4x4 quaternion covariance via analytical Jacobian (first-order approximation) |
| `twist.covariance` | 6x6 | Exact from state covariance |

Also publishes:
- `/accel/filtered` (`geometry_msgs/AccelWithCovarianceStamped`) — body-frame acceleration
- TF: `odom -> base_link`

---

## 8. What Can Go Wrong

### EKF not publishing (0 Hz on `/ekf/odometry`)

No sensor data reaching the filter.
```bash
ros2 topic hz /ov_msckf/odomimu    # VIO alive?
ros2 topic hz /aruco/drone_pose     # Vision alive?
ros2 node list | grep ekf           # Node running?
```

### Orientation stuck at identity or zero

| Symptom | Cause | Fix |
|---------|-------|-----|
| Orientation is (0,0,0,0) | Filter not initialized — no measurement received yet | Wait for VIO to start publishing (2-5s after camera data flows) |
| Orientation is (0,0,0,1) always | Orientation indices disabled in config | Verify `odom0_config` indices 3-6 are `true` |
| Orientation is (0,0,0,1) always | VIO publishing zero quaternion | `ros2 topic echo /ov_msckf/odomimu --field pose.pose.orientation --once` |

### Position drifts or jumps

1. **No ArUco markers visible** — position drifts with VIO quality. Expected without markers.
2. **Mahalanobis rejection** — vision covariance too tight relative to accumulated drift. Increase `pose0_rejection_threshold` in config.
3. **Check `odom0_differential`** — must be `false` for absolute pose fusion.

### Quaternion norm drifting from 1.0

Should not happen (normalized every step). If it does: check for NaN in sensor data. Enable `debug: true` and inspect `/tmp/ekf_debug.txt`.

### TF transform errors ("Could not transform measurement")

The EKF needs transforms between sensor frames and `base_link`. Check:
```bash
ros2 run tf2_tools view_frames    # Visual TF tree
```
Common mismatch: sensor publishes in `camera_frame` but TF only has `camera_link`.

### Build takes forever

Only rebuild `robot_localization` when you changed its source. For `ekf` package alone:
```bash
colcon build --symlink-install --packages-select ekf --cmake-args -DBUILD_TESTING=OFF
```

---

## 9. Files Modified in `robot_localization`

These are the files we changed in the `robot_localization` submodule. If something breaks in the filter itself, look here:

| File | What changed |
|------|-------------|
| `include/.../filter_common.hpp` | State enum: 16 states, Qw/Qx/Qy/Qz at indices 3-6. `STATE_SIZE=16`. Backward-compat aliases (`StateMemberRoll=StateMemberQw`, etc.). `ORIENTATION_SIZE` stays 3 (see below). |
| `src/ekf.cpp` | **Full rewrite of `predict()`** — DroneModel physics: quaternion kinematics, gravity coupling, angular rate decay, analytical Jacobian. |
| `src/filter_base.cpp` | `wrapStateAngles()`: Euler wrapping replaced with quaternion normalization. `reset()`: 16x16 matrices. |
| `src/ros_filter.cpp` | Output: writes quaternion directly, projects covariance to RPY for ROS compat. Input: reads quaternion from measurements, hemisphere dot-product flip, `dQuatDRPY()` Jacobian for covariance conversion. Bug fix: `twist_update_vec` range corrected to exclude Qz. |
| `src/ros_filter_utilities.cpp` | `stateToTF()`/`TFtoState()`: direct quaternion read/write instead of RPY round-trip. |

### ORIENTATION_SIZE = 3 — DO NOT CHANGE

Despite 4 quaternion components, this constant must stay 3. It sizes:
- `sensor_msgs/Imu::orientation_covariance` (float64[9] = 3x3)
- 6x6 `pose.covariance` arrays (3 pos + 3 ori)
- Eigen `block<3,3>(3,3)` on 6x6 matrices

Changing to 4 causes out-of-bounds crashes.

### Submodule commit rules

- Always commit inside the submodule first, then commit the updated reference in main repo
- **Never `git add .`** inside the submodule — add files by path
- Run `git checkout main` inside the submodule before changes (defaults to detached HEAD)
- Only `ekf_node` is supported — the UKF uses backward-compat aliases that map quaternion slots to Euler assumptions (wrong physics, no crash)

---

## 10. Topic Map

```
┌─────────────────┐     /cam0/image_raw      ┌──────────────────┐
│ Gazebo Harmonic  ├──────────────────────────►│ aruco_pose_est.  │
│ (gz_my_drone)    │     /imu0                │                  │
│                  ├──────────┐               └────────┬─────────┘
└─────────────────┘           │                        │
                              ▼                        │ /aruco/drone_pose
                    ┌──────────────────┐               │ (PoseWithCovStamped)
                    │ OpenVINS (VIO)   │               │
                    │ ov_msckf         │               │
                    └────────┬─────────┘               │
                             │                         │
                             │ /ov_msckf/odomimu       │
                             │ (Odometry)              │
                             ▼                         ▼
                    ┌──────────────────────────────────────────┐
                    │         EKF (robot_localization)         │
                    │    16-state quaternion DroneModel         │
                    └──────────────────┬───────────────────────┘
                                       │
                                       │ /ekf/odometry (Odometry, 100Hz)
                                       │ /accel/filtered
                                       │ TF: odom → base_link
                                       ▼
                              ┌──────────────────┐
                              │ drone_controller  │
                              │ (MPC / RL)        │
                              └──────────────────┘
```

---

## 11. Quick Debug Checklist

If the pipeline is not working, go through these in order:

- [ ] `source install/setup.bash` in every terminal
- [ ] PX4 SITL is running (`make px4_sitl gz_my_drone`)
- [ ] `ros2 topic list` shows `/ov_msckf/odomimu` and `/aruco/drone_pose`
- [ ] `ros2 topic hz /ov_msckf/odomimu` shows > 0 Hz
- [ ] `ros2 topic hz /ekf/odometry` shows ~100 Hz
- [ ] Orientation is not (0,0,0,0) — filter has initialized
- [ ] TF tree includes `odom -> base_link` (`ros2 run tf2_tools view_frames`)
- [ ] Controller is receiving EKF output (`ros2 topic echo /ekf/odometry --once`)
