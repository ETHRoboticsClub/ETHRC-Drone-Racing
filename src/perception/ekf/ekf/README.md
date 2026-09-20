# EKF Package

## Overview

Extended Kalman Filter for drone state estimation using a **16-state quaternion model** with custom drone physics. Built on top of a modified `robot_localization` (submodule at `src/perception/ekf/robot_localization/`).

The filter replaces the stock 15-state Euler EKF with quaternion kinematics, gravity coupling, and angular rate decay to eliminate gimbal-lock singularities and improve predict-step accuracy during aggressive maneuvers.

**State vector** (16 states, ENU world frame):
```
[X, Y, Z, Qw, Qx, Qy, Qz, Vx, Vy, Vz, Vroll, Vpitch, Vyaw, Ax, Ay, Az]
```

> **First time running the pipeline?** Read [SETUP.md](SETUP.md) for the full integration guide.

## Input Topics

### VIO (Odometry)
- **`/ov_msckf/odomimu`**
  - **Type**: `nav_msgs/msg/Odometry`
  - **Description**: Visual-inertial odometry — position, orientation, velocity, angular rates
  - **Frequency**: IMU rate (typically 200-400 Hz)

### Vision (Absolute Pose)
- **`/aruco/drone_pose`**
  - **Type**: `geometry_msgs/msg/PoseWithCovarianceStamped`
  - **Description**: Absolute pose from ArUco marker detection — serves as drift correction
  - **Frequency**: Variable (depends on marker visibility)

## Output Topics

- **`/ekf/odometry`**
  - **Type**: `nav_msgs/msg/Odometry`
  - **Description**: Fused state estimate. Pose in world (ENU) frame, twist in body frame.
  - **Frequency**: 100 Hz

- **`/accel/filtered`**
  - **Type**: `geometry_msgs/msg/AccelWithCovarianceStamped`
  - **Description**: Estimated body-frame linear acceleration (from gravity coupling model)

- **TF**: `odom -> base_link`

## Build & Launch

```bash
# Build
colcon build --symlink-install --packages-select ekf --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash

# Launch (standalone EKF node)
ros2 launch ekf rl_ekf.launch.xml

# Launch (full perception pipeline: bridges + ArUco + VIO + EKF)
ros2 launch launch/perception.launch.py
```

## Configuration

Parameters are in `config/rl_ekf.yaml`. All sensor `_config` arrays use **16 booleans** matching the state vector:

```yaml
# [x, y, z, qw, qx, qy, qz, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az]
odom0_config: [true, true, true,   true, true, true, true,   true, true, true,
               true, true, true,  false, false, false]
```

## Test Infrastructure

The `mock_publisher` node generates synthetic sensor data for controlled testing without Gazebo/PX4:

```bash
# Run a test scenario (rest, const_vel, turn, accel, sprint, slalom, pitch_dive, dropout)
ros2 launch ekf scenario_record.launch.py scenario:=turn duration:=30.0

# Hard tests (VIO orientation disabled, sparse 5Hz vision)
ros2 launch ekf hard_scenario_record.launch.py scenario:=sprint duration:=60.0
```

Analysis notebooks: `plots/compare_ekf.ipynb` (vs legacy filter) and `plots/hard_ekf.ipynb` (hard scenarios).
