# Perception Stack Overview

The perception stack is responsible for providing a reliable, drift-free state estimate of the drone in the world frame. It achieves this by fusing high-frequency relative odometry with sparse absolute pose corrections.

## High-Level Architecture

The system follows a tiered architecture for pose estimation:

```mermaid
graph TD
    subgraph Sensors
        Cam["Camera (Mono/Stereo)"]
        IMU["IMU (Accel/Gyro)"]
        Vicon["Vicon (Optional)"]
    end

    subgraph "Local Odometry"
        VIO["OpenVINS (VIO)"]
    end

    subgraph "Absolute Corrections"
        Gate["Gate Detection (WIP)"]
    end

    subgraph "Fusion Layer"
        EKF["EKF (robot_localization)"]
    end

    Cam --> VIO
    IMU --> VIO
    VIO -->|"/ov_msckf/odomimu (Odom)"| EKF
    
    Cam --> Gate
    Gate -->|"/gate_detection/drone_pose (Pose)"| EKF
    
    
    Vicon -->|"(Indoor GT)"| EKF
    
    EKF -->|"/odometry/filtered"| Control["Control & Planning"]
```

## Topic Mappings

The following table summarizes the data flow between perception modules.

| Source Module | Topic Name | Message Type | Frequency | Consumer |
| :--- | :--- | :--- | :--- | :--- |
| **Sensors** | `/cam0/image_raw` | `sensor_msgs/Image` | 30 Hz | VIO, Gate Detection |
| **Sensors** | `/imu0` | `sensor_msgs/Imu` | 200 Hz | VIO |
| **OpenVINS** | `/ov_msckf/odomimu` | `nav_msgs/Odometry` | 100 Hz | EKF |
| **Gate Detection** | `/gate_detection/drone_pose` | `PoseWithCovarianceStamped` | 10-30 Hz | EKF |
| **Vicon** | `/vicon/drone/drone` | `geometry_msgs/Transform` | 100 Hz | EKF |
| **EKF** | `/odometry/filtered` | `nav_msgs/Odometry` | 100 Hz | Control |

## Coordinate Systems

The perception stack utilizes the standard ROS 2 (REP-103) coordinate conventions:

- **`world` Frame**: Global fixed frame. Its origin is typically defined at the start of the mission.
- **`odom` Frame**: A locally stable frame provided by the EKF. It is continuous (no jumps) but may drift relative to the `world` frame over long durations if absolute fixes are lost.
- **`base_link` Frame**: The center of mass of the drone.
    - **X-axis**: Forward
    - **Y-axis**: Left
    - **Z-axis**: Up

## Detailed Workings

### 1. Visual-Inertial Odometry (OpenVINS)
The **VIO** component uses the `open_vins` library to fuse camera images and IMU data.
- **Formulation**: Implements an **Extended Kalman Filter (EKF)** with a **Multi-State Constraint Kalman Filter (MSCKF)**.
- **Constraint-Based**: It avoids estimating feature positions in the main filter state, making it highly efficient for high-speed flight.
- **Online Calibration**: Online-calibrates the rotation and translation between the Camera and IMU.
- **Output**: Published as `nav_msgs/Odometry` on `/ov_msckf/odomimu`.

### 2. Gate Detection & PnP (WIP)
The **Gate Detection** component identifies racing gates and estimates the drone's pose relative to them.
- **Deep Learning**: Uses a **U-Net** based segmentation network (**GateNet**) to detect gate masks.
- **QuAdGate**: Extracts sub-pixel corner candidates and matches them using geometric descriptors.
- **PnP Solver**: Uses `cv2.solvePnP` to find the absolute translation and rotation from the known gate dimensions to the observed pixel coordinates.
- **Output**: Published as `geometry_msgs/PoseWithCovarianceStamped` on `/gate_detection/drone_pose`.


### 3. Fusion Logic (The EKF)
The system uses `robot_localization` with **Smooth Lagged Data** support.
- **Rewind/Fast-Forward**: Vision processing takes time (50-100ms). The EKF maintains a history of past states, "rewinds" to the image timestamp to fuse the delayed measurement, and then re-propagates the state to the current time.
- **Output**: The definitive state estimate on `/odometry/filtered`.

## Modules

For deeper technical details on individual components, see:
- [VIO (OpenVINS)](vio.md)
- [Blackbox Driver (IMU/RPM)](blackbox.md)
- [Gate Detection](gate_detection.md)
- [Sensor Fusion (EKF)](EKF.md)
- [Vicon Tracking](vicon.md)
