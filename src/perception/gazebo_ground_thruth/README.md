# Gazebo Ground Truth Visualization

A simple ROS2 node that visualizes the drone's ground truth trajectory from Gazebo simulation as a trail of markers in RViz.

## Input Topics

- **`/odometry`**
  - **Type**: `nav_msgs/msg/Odometry`
  - **Description**: Ground truth odometry from Gazebo (bridged from `/drone/odometry`)

## Output Topics

- **`/Gazebo_Ground_Thruth`**
  - **Type**: `visualization_msgs/msg/Marker`
  - **Description**: Blue sphere markers (0.08m diameter) showing the drone's path for visualization in RViz

## Parameters

- **`world_frame`** (default: `"world"`)
  - Frame ID for the visualization markers

## Note

This node is **visualization only**. If you need ground truth pose data, subscribe directly to `/odometry`