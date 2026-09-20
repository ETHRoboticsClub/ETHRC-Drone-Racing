# Extended Kalman Filter (EKF)

**Page Owner:** [Oliver Calvet](https://github.com/oliomap)

---

## Overview

The EKF package provides sensor fusion for drone state estimation. It fuses visual-inertial odometry (VIO) from OpenVINS with absolute pose measurements from **Gate Detection** to produce a robust, high-frequency state estimate. The filter is built on a **modified fork of `robot_localization`** (submodule at `src/perception/ekf/robot_localization/`) that replaces the stock 15-state Euler EKF with a **16-state quaternion model driven by quadrotor physics** — quaternion kinematics, gravity-coupled acceleration, and first-order angular-rate decay in the predict step. The quaternion state eliminates the gimbal-lock singularity of the Euler formulation and improves predict-step accuracy during aggressive maneuvers. The full physics derivation lives in the package `SETUP.md`.

## Input Topics

The EKF system subscribes to the following ROS 2 topics. Orientation is represented natively as a unit quaternion in the filter's 16-dimensional state vector: `[X, Y, Z, Qw, Qx, Qy, Qz, Vx, Vy, Vz, Vroll, Vpitch, Vyaw, Ax, Ay, Az]` — world-frame (ENU) position, body→world quaternion, body-frame linear velocity, body angular rates, and body-frame acceleration.

### VIO Odometry
- **Topic**: `/ov_msckf/odomimu`
- **Type**: `nav_msgs/msg/Odometry`
- **Description**: Odometry from OpenVINS containing pose (position + orientation) and velocity estimates
- **Used for Update**: position (x, y, z), orientation quaternion (qw, qx, qy, qz), linear velocity (vx, vy, vz), angular rates (vroll, vpitch, vyaw)

### Gate Detection Pose
- **Topic**: `/gate_detection/drone_pose`
- **Type**: `geometry_msgs/msg/PoseWithCovarianceStamped`
- **Description**: Absolute pose measurements from Gate Detection
- **Used for Update**: position (x, y, z) and orientation quaternion (qw, qx, qy, qz)

### Control Input (Optional)
- **Topic**: `/control/command` (remapped from `cmd_vel`)
- **Type**: `geometry_msgs/msg/Twist`
- **Description**: Velocity commands used for motion prediction (currently disabled)

## Output Topics

The EKF system publishes the following ROS 2 topics:

### Fused Odometry
- **`/ekf/odometry`**
  - **Type**: `nav_msgs/msg/Odometry`
  - **Description**: Fused state estimate containing pose and velocity with covariances
  - **Frequency**: 100 Hz (configurable via `frequency` parameter)

### Acceleration
- **`/accel/filtered`**
  - **Type**: `geometry_msgs/msg/AccelWithCovarianceStamped`
  - **Description**: Estimated linear acceleration

### Transforms (TF)
- **`/tf`**
  - **Description**: Transform from `world` frame to `base_link` frame via `odom` frame
  - **Frequency**: 100 Hz

## Brief Explanation

The sensor fusion pipeline works as follows:

1. **Prediction Step**: The filter propagates the state estimate forward in time using a **quadrotor physics model** (not a generic constant-velocity model): quaternion kinematics integrate the body angular rates, gravity coupling drives the body-frame acceleration as a function of attitude, and a first-order decay relaxes the angular rates between measurements. The prediction runs at 100 Hz to maintain smooth state estimates.

2. **VIO Update**: When odometry messages arrive from OpenVINS (`/ov_msckf/odomimu`), the filter incorporates full pose and velocity information (position, quaternion orientation, linear velocity, angular rates) to update the state estimate.

3. **Vision Update**: Absolute pose measurements from Gate Detection (`/gate_detection/drone_pose`) provide global corrections to prevent drift. Only pose states (position and quaternion orientation) are fused from this source.

4. **Delayed Measurement Handling**: The filter maintains a 0.5-second state history buffer (`smooth_lagged_data: true`) to properly handle out-of-order or delayed sensor measurements.

5. **Output**: The fused state estimate is published as odometry and TF transforms for use by other modules.

## Configuration

The filter parameters are configured in `config/rl_ekf.yaml`. Key settings include:

| Parameter | Value | Description |
|-----------|-------|-------------|
| `frequency` | 100.0 Hz | Filter update rate |
| `sensor_timeout` | 0.1 s | Timeout before sensor is considered stale |
| `publish_tf` | true | Publish TF transforms |
| `two_d_mode` | false | Full 3D state estimation |
| `history_length` | 0.5 s | State history for delayed measurements |

### Frame Configuration

| Frame | Value | Description |
|-------|-------|-------------|
| `map_frame` | `world` | Global fixed reference frame for the environment |
| `odom_frame` | `odom` | Continuous odometry frame (can drift over time) |
| `base_link_frame` | `base_link` | Frame rigidly attached to the drone body |
| `world_frame` | `world` | Reference frame for filter operation |

Setting `world_frame: world` (same as `map_frame`) configures the filter to operate in the global map frame. This is required because we fuse absolute pose measurements from **Gate Detection**, which provide global corrections and eliminate drift. If `world_frame` were set to `odom`, the filter would only output continuous odometry without incorporating these global corrections.

## Launch EKF in Gazebo Simulation

```bash
ros2 launch ekf rl_ekf.launch.xml
```

This starts the `robot_localization` EKF node with fused odometry published on `/ekf/odometry`.
