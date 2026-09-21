# Perception

ROS 2 packages for sensing and state estimation. The documentation lives in the wiki under **Perception** (`docs/docs/perception/`, start at `index.md`); each subsystem has a *Technical* page (what it is) and a *Workflow* page (how to run it).

| Package | Purpose | Status |
| :--- | :--- | :--- |
| `imx219_argus_camera` | GStreamer driver for the IMX219 CSI camera (Jetson only); used for calibration recordings. The flight bringup uses `drone_control/launch/nitros_camera.launch.py` instead | Production (calibration) |
| `betaflight_blackbox_driver` | Betaflight blackbox UART decoder → `/imu0`, `/motor_rpm`, `/battery`; MSP fallback node | Production |
| `VIO/open_vins` | OpenVINS fork (submodule) | Experimental |
| `ekf/ekf`, `ekf/robot_localization` | 16-state quaternion EKF on a robot_localization fork (submodule) | Experimental |
| `gate_detection` | Gate detector + PnP pose (v3.4 hybrid model; AIGP colour detector; v1 offline tools) | Experimental |
| `aigp_camera_driver`, `aigp_imu_driver` | AI Grand Prix simulator adapters → `/cam0/*`, `/imu0` | Simulation |
| `camera_calibration/` | IMX219 intrinsics results and checkerboard (not a package) | Data |
| `gazebo_ground_thruth`, `pointcloud_gt` | RViz ground-truth markers from Gazebo odometry | Simulation |
| `archive/ArUco_estimation` | ArUco pose estimation, `COLCON_IGNORE` | Deprecated |

Vicon lives in `src/ros2-vicon-receiver`; stack-level launch files in `launch/`; helper scripts in `scripts/calibration/` and `scripts/perception/`.

Build:

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --packages-up-to betaflight_blackbox_driver vicon_receiver ov_msckf ekf gate_detection
```
