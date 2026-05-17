# Gate Detection & PnP Pose Estimation (WIP)

**Page Owner:** [Mahbod Tajdini](https://github.com/Ace3Z)

---

## Overview

The Gate Detection module is a neural-network based vision system designed to provide absolute pose corrections for autonomous drone racing. It replaces the legacy ArUco-based system by detecting specialized racing gates and using the known geometry of the gates to calculate the drone's position and orientation via **Perspective-n-Point (PnP)**.

> [!IMPORTANT]
> This module is currently under active development (**Work-in-Progress**). The ROS 2 node and PnP solver are in a functional but experimental state.

## System Architecture

The pipeline consists of three primary stages:

1.  **Segmentation (GateNet)**: A U-Net style convolutional neural network that takes a raw camera image and produces a binary segmentation mask of the gate.
2.  **Corner Extraction (QuAdGate)**: A geometric processing algorithm that:
    *   Finds lines in the segmentation mask using LSD (Line Segment Detector).
    *   Generates sub-pixel corner candidates from line intersections.
    *   Matches candidates to known gate corner types (Outer TL/TR/BR/BL and Inner TL/TR/BR/BL) using shape descriptors.
3.  **Pose Estimation (PnP)**: Uses `cv2.solvePnP` to map the 2D pixel coordinates of the detected corners to their 3D positions in the gate's local coordinate frame.

## ROS 2 Integration

### Nodes

#### `detector_node`
The primary real-time node that runs the full inference pipeline.

*   **Subscribes to**: `/cam0/image_rgb` (`sensor_msgs/Image`) by default
*   **Publishes to**: `/gate_detection/drone_pose` (`geometry_msgs/PoseWithCovarianceStamped`)
*   **Parameters**:
    *   `checkpoint`: Path to the `.pth` model file.
    *   `camera_topic`: The image topic to subscribe to.
    *   `output_topic`: The pose topic to publish.

The detector converts incoming frames to `rgb8` before inference.
`/cam0/image_raw` is `mono8`, so it does not contain real color information.
Use `/cam0/image_rgb` for real color. `/cam0/image_h264` is compressed video,
not detector input. See [Jetson Camera Pipeline](camera_pipeline.md).

### Launching

To launch the gate detection system within the perception stack:

```bash
ros2 launch gate_detection gate_detection.launch.py
```

In the full racing bringup, gate detection is currently disabled by default
until the package is production-ready:

```bash
ros2 launch drone_control racing_bringup.launch.py \
  msp_bench_mode:=true \
  use_gate_detection:=true \
  gate_camera_topic:=/cam0/image_rgb
```

## Development Tools

The package includes standalone scripts for dataset inspection and performance validation:

*   **`inference_standalone`**: Runs inference on a directory of images and generates a visual debug plot showing segmentation, line detection, and corner matching.
*   **`viewer_standalone`**: An interactive tool to review and clean synthetic datasets.

## Known Issues & TODOs

- [ ] **Camera Intrinsics**: Currently uses hardcoded placeholder values; needs to subscribe to `camera_info`.
- [ ] **Gate Dimensions**: Dynamic loading of gate dimensions from a map configuration (similar to old ArUco YAMLs).
- [ ] **Temporal Filtering**: Integration of a tracker to handle intermittent detections and motion blur.
- [ ] **Performance**: Optimization for Jetson Orin NX (TensorRT conversion).
