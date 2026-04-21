# ArUco Pose Estimation (DEPRECATED)

!!! warning "DEPRECATED"
    **This module is officially deprecated and is no longer part of the active build system.**
    It has been replaced by the [Gate Detection](gate_detection.md) system. This documentation is maintained in the **Deprecated** section for historical reference only.

**Page Owner:** [Xander Yap](https://github.com/xanderyap) & [Jules Schmutz](https://github.com/juliomathis)

## Overview

We provide an ArUco marker [DICT_5x5_1000] detection system serving as an initial gate detection pipeline and as a fallback ground truth method. The markers have to be physically placed in the environment, subsequently ground truth location and the size of the markers have to be parsed into the system.

## Input Topics

The ArUco pose estimation system subscribes to the following ROS2 topics:

### Camera Images
- **`/cam0/image_raw`**

  - **Type**: `sensor_msgs/msg/Image`
  - **Description**: Raw camera images from a single camera
  - **Frequency**: Typically 20-30 Hz

## Output Topics

The ArUco pose estimations system publishes the following ROS 2 topics (under namespace /aruco by default):

### Diagnostic

- **`/aruco/camera_pose`**

    - **Type**: `geometry_msgs::msg::PoseStamped`
    - **Description**: Publishes the pose and location of the **camera**

- **`/aruco/drone_pose`**

    - **Type**: `geometry_msgs::msg::PoseWithCovarianceStamped`
    - **Description**: Publishes the pose and location of the **drone**

- **`/aruco/visualization_markers`**

    - **Type**: `visualization_msgs::msg::Marker`
    - **Description**: Publishes point cloud markers (for e.g. rviz2) to show the pose and location of the drone

- **`/aruco/diagnostics`**

    - **Type**: `diagnostic_msgs::msg::DiagnosticArray`
    - **Description**: Publishes the values in the terminal

## Brief explanation

This section provides a short walkthrough and explanation of the detection system:

1. **Configuration:** as mentioned in the overview, parse the marker **ID**, **pose**, and **size** into a YAML file in `config/`.

2. **Detection:** we utilize the marker detection library from OpenCV and enhance the performance with Lucas-Kanade optical flow and a smoothening factor.

3. **Visual Update:** the borders of the detected markers will be highlighted.

4. **Output:** The estimated pose is published as a 6-DoF transform consisting of:
   - Position: *(x, y, z)*
   - Orientation: quaternion *(x, y, z, w)*

## Launch ArUco Marker Estimation in Gazebo Simulation

```
ros2 launch aruco_pose_estimation aruco_pose_estimation.launch.py
```

