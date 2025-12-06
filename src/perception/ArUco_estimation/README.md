# ArUco Pose Estimation

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


