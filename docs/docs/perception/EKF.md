# Extended Kalman Filter (EKF)

**Page Owner:** [Oliver Calvet](https://github.com/oliomap)

---

## Overview

The EKF package provides sensor fusion for drone state estimation by leveraging the Robot Localization package's Extended Kalman Filter implementation. The filter fuses visual-inertial odometry (VIO) from OpenVINS with absolute pose measurements from ArUco marker detection to produce a robust, high-frequency state estimate. The current implementation uses the standard `robot_localization` package without drone-specific model adaptations.

## Input Topics

The EKF system subscribes to the following ROS 2 topics. Orientation data arrives as quaternions but is internally converted to roll, pitch, yaw (RPY) for the filter's 15-dimensional state vector: `[x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az]`.

### VIO Odometry
- **Topic**: `/ov_msckf/odomimu`
- **Type**: `nav_msgs/msg/Odometry`
- **Description**: Odometry from OpenVINS containing pose (position + orientation) and velocity estimates
- **Used for Update**: x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw

### ArUco Pose
- **Topic**: `/aruco/drone_pose`
- **Type**: `geometry_msgs/msg/PoseWithCovarianceStamped`
- **Description**: Absolute pose measurements from ArUco marker detection
- **Used for Update**: x, y, z, roll, pitch, yaw

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

1. **Prediction Step**: The filter propagates the state estimate forward in time using a constant-velocity motion model. The prediction runs at 100 Hz to maintain smooth state estimates.

2. **VIO Update**: When odometry messages arrive from OpenVINS (`/ov_msckf/odomimu`), the filter incorporates full pose and velocity information (x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw) to update the state estimate.

3. **Vision Update**: Absolute pose measurements from ArUco marker detection (`/aruco/drone_pose`) provide global corrections to prevent drift. Only pose states (x, y, z, roll, pitch, yaw) are fused from this source.

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

Setting `world_frame: world` (same as `map_frame`) configures the filter to operate in the global map frame. This is required because we fuse absolute pose measurements from ArUco markers, which provide global corrections and eliminate drift. If `world_frame` were set to `odom`, the filter would only output continuous odometry without incorporating these global corrections.

## Launch EKF in Gazebo Simulation

```bash
ros2 launch ekf rl_ekf.launch.xml
```

This starts the `robot_localization` EKF node with fused odometry published on `/ekf/odometry`.
