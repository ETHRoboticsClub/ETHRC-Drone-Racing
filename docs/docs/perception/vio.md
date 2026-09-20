# Visual Inertial Odometry (VIO)

**Page Owner:** [Giacomo Piatelli](https://github.com/CornelioP)

---

## Overview

OpenVINS is a state-of-the-art filter-based visual-inertial odometry system that estimates the 6-DOF pose (position and orientation) and velocity of the drone by fusing camera images with IMU measurements. The system uses an Extended Kalman Filter (EKF) with Multi-State Constraint Kalman Filter (MSCKF) formulation to efficiently track visual features and estimate the state without directly estimating feature positions in the filter state.

## Input Topics

The VIO system subscribes to the following ROS 2 topics:

### IMU Data
- **Topic**: `/imu0` (default, configurable via `topic_imu` parameter)
- **Type**: `sensor_msgs/msg/Imu`
- **Description**: IMU measurements containing linear acceleration and angular velocity
- **Frequency**: Typically 200-400 Hz 

### Camera Images
- **Monocular Mode**:
  - **Topic**: `/cam0/image_raw` (default, configurable via `topic_camera0` parameter)
  - **Type**: `sensor_msgs/msg/Image`
  - **Description**: Mono `mono8` images from the IMX219 racing pipeline
  - **Frequency**: 60 Hz on the IMX219 Jetson racing pipeline

The camera path is shared with optional H.264 streaming. OpenVINS should stay on
`/cam0/image_raw`; do not subscribe VIO to `/cam0/image_rgb` or
`/cam0/image_h264`. See [Jetson Camera Pipeline](camera_pipeline.md).

- **Stereo Mode**:
  - **Topics**: `/cam0/image_raw` and `/cam1/image_raw` (default, configurable)
  - **Type**: `sensor_msgs/msg/Image`
  - **Description**: Synchronized stereo image pairs
  - **Frequency**: Typically 20-30 Hz (synchronized)

## Output Topics

The VIO system publishes the following ROS 2 topics (under namespace `/ov_msckf` by default):

### Pose and Odometry
- **`/ov_msckf/poseimu`**
  - **Type**: `geometry_msgs/msg/PoseWithCovarianceStamped`
  - **Description**: Current pose estimate (position + orientation) with covariance
  - **Frequency**: ~20 Hz (tracking frequency, configurable via `track_frequency` parameter)

- **`/ov_msckf/odomimu`**
  - **Type**: `nav_msgs/msg/Odometry`
  - **Description**: High-frequency odometry message containing pose, velocity, and covariances
  - **Frequency**: Up to IMU rate (200-400 Hz) for high-frequency pose estimates

- **`/ov_msckf/pathimu`**
  - **Type**: `nav_msgs/msg/Path`
  - **Description**: Historical trajectory path
  - **Frequency**: ~20 Hz

### Visual Features
- **`/ov_msckf/points_msckf`**
  - **Type**: `sensor_msgs/msg/PointCloud2`
  - **Description**: 3D point cloud of MSCKF tracked features
  - **Frequency**: ~20 Hz

- **`/ov_msckf/points_slam`**
  - **Type**: `sensor_msgs/msg/PointCloud2`
  - **Description**: 3D point cloud of SLAM features (features in state vector)
  - **Frequency**: ~20 Hz

- **`/ov_msckf/trackhist`**
  - **Type**: `sensor_msgs/msg/Image`
  - **Description**: Visualization image showing tracked features
  - **Frequency**: 20 Hz

### Transforms (TF)
- **`/tf`**
  - **Description**: Transform from `global` frame to `imu` frame, and from `imu` to `cam0`, `cam1`, etc.
  - **Frequency**: ~20 Hz

## Brief Explanation

The VIO pipeline works as follows:

1. **Feature Extraction & Tracking**: Visual features are detected and tracked across consecutive camera frames using KLT (Kanade-Lucas-Tomasi) tracking.

2. **IMU Propagation**: Between camera frames, the IMU measurements are integrated to propagate the state estimate forward in time, providing high-frequency motion estimates.

3. **Visual Update**: When new camera frames arrive, the tracked features are used to update the state estimate. The MSCKF formulation allows using visual constraints without explicitly estimating 3D feature positions in the filter state, making it computationally efficient.

4. **State Estimation**: The EKF maintains estimates of:
   - Pose (position and orientation)
   - Velocity
   - IMU biases
   - Camera-IMU calibration (extrinsics)
   - Camera intrinsics (if enabled)

5. **Output**: The estimated pose, velocity, and trajectory are published to ROS 2 topics for use by other modules.

## Launch OpenVINS in Gazebo Simulation

```bash
ros2 launch ov_msckf subscribe.launch.py \
    config:=gazebo_sim \
    use_stereo:=false \
    config_path:=/home/drone_dev/ros_ws/src/perception/VIO/open_vins/config/gazebo_sim/estimator_config.yaml \
    use_stereo:=false \
    max_cameras:=1
```

> [!TIP]
> You can also use the top-level orchestration launch file which handles these paths automatically:
> ```bash
> ros2 launch launch/perception.launch.py
> ```

> [!IMPORTANT]
> It is important to enforce the `use_stereo` and `max_cameras` parameters, otherwise, the system may encounter a segmentation fault.

## Debugging Segmentation Faults
In case of segmentation error to be able to debug add the following line in the `ov_msckf/launch/subscribe.launch.py` file:

```
    node1 = Node(
        package="ov_msckf",
        executable="run_subscribe_msckf",
        condition=IfCondition(LaunchConfiguration("ov_enable")),
        namespace=LaunchConfiguration("namespace"),
        output='screen',
        prefix=['gdb -ex run -ex "bt full" -ex quit --args'], # ADD THIS LINE
        parameters=[
            {"verbosity": LaunchConfiguration("verbosity")},
            {"use_stereo": ParameterValue(LaunchConfiguration("use_stereo"), value_type=bool)},
            {"max_cameras": ParameterValue(LaunchConfiguration("max_cameras"), value_type=int)},
            {"save_total_state": ParameterValue(LaunchConfiguration("save_total_state"), value_type=bool)},
            {"config_path": config_path},
            {"record_timing_information": ParameterValue(LaunchConfiguration("dotime"), value_type=bool)},
            {"record_timing_filepath": LaunchConfiguration("path_time")},
        ],
    )
```

This way we are going to enable gdb, which give us the possibility of trace back which function call lead to the segmentation fault
