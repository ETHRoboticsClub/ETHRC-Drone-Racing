# Perception

Sensors and state estimation for the racing drones: camera, IMU, motion capture, visual-inertial odometry, sensor fusion and gate detection.

Every subsystem has two pages. **Technical** answers *what is this* — nodes, topics, frames, parameters, files, limitations. **Workflow** answers *how do I run it* — build, launch, verify, debug.

## Where to start

1. [Overview → Technical](overview/technical.md) — the data-flow diagram, the frame conventions, and what actually runs in flight. Fifteen minutes; everything else assumes it.
2. [Overview → Workflow](overview/workflow.md) — build the packages and bring the drone up on the bench. Do this once before touching any subsystem.
3. The two pages of the subsystem you will work on, in the order the table lists them.

## Subsystems

| Subsystem | Code | Status | Pages |
| :--- | :--- | :--- | :--- |
| Overview: data flow, frames, bringup | `launch/`, `drone_control/launch/racing_bringup.launch.py` | — | [Technical](overview/technical.md) · [Workflow](overview/workflow.md) |
| Camera (IMX219 on Jetson) | `imx219_argus_camera`, `drone_control/launch/nitros_camera.launch.py` | Production | [Technical](camera/technical.md) · [Workflow](camera/workflow.md) |
| IMU (Betaflight blackbox over UART) | `betaflight_blackbox_driver` | Production | [Technical](imu/technical.md) · [Workflow](imu/workflow.md) |
| Vicon motion capture | `src/ros2-vicon-receiver` | Production — the pose source in flight | [Technical](vicon/technical.md) · [Workflow](vicon/workflow.md) |
| VIO (OpenVINS) | `VIO/open_vins` | Experimental — not calibrated for the real camera | [Technical](vio/technical.md) · [Workflow](vio/workflow.md) |
| EKF (robot_localization fork) | `ekf`, `ekf/robot_localization` | Experimental — off by default in flight | [Technical](ekf/technical.md) · [Workflow](ekf/workflow.md) |
| Gate detection | `gate_detection` | Experimental — model checkpoint not in the repo | [Technical](gate_detection/technical.md) · [Workflow](gate_detection/workflow.md) |
| AIGP simulator drivers | `aigp_camera_driver`, `aigp_imu_driver` | Simulation only | [Technical](aigp_drivers/technical.md) · [Workflow](aigp_drivers/workflow.md) |
| Camera calibration | `camera_calibration/`, `scripts/calibration/` | Intrinsics done · camera–IMU open | [Technical](calibration/technical.md) · [Workflow](calibration/workflow.md) |
| Gazebo ground-truth markers | `gazebo_ground_thruth`, `pointcloud_gt` | Simulation only | [Overview → Technical](overview/technical.md#simulation-helpers) |
| ArUco pose estimation | `archive/ArUco_estimation` | Deprecated, not built | [ArUco](../deprecated/ArUco.md) |

All package paths are relative to `src/perception/` unless they start with `src/` or `scripts/`.

## What matters, and in which order

The stack exists to fly without external infrastructure: camera + IMU → VIO → EKF, corrected by gate detections. Today none of that is in the loop. The drone flies on **Vicon + IMU** through `state_bridge_node` ([details](overview/technical.md#what-is-in-the-flight-loop)); VIO, EKF and gate detection are launch options that default to off. Read the subsystems with that in mind.

**Flight-critical, stable.** Camera, IMU, Vicon. A regression here grounds the drone, so learn their interfaces and health checks first even if you will not work on them. Each carries one open item worth knowing before you build on it: the camera's mono and RGB streams differ by a 180° rotation, the IMU has no hardware timestamp, Vicon has no latency compensation.

**The open work.** VIO, EKF and gate detection, in a dependency chain: VIO cannot work until the camera–IMU calibration is finished and the real camera intrinsics are in its config; the EKF has been validated only with Vicon standing in for VIO; gate detection needs a model checkpoint, a QoS fix and a map of the lab before it can feed the EKF. If you want to make the most difference with the least code, start with [Calibration → Workflow](calibration/workflow.md): the Kalibr run is the first blocker in that chain and needs no new software.

**Low priority.** The AIGP simulator drivers were written for the AI Grand Prix virtual qualifier, which is over. They are kept so the simulator still runs, but they were never fully integrated — the simulator bringup takes its IMU from a different node, the camera and IMU stamps are on different clocks, and the gate detector used there is a colour heuristic tuned to the simulator's gates and camera. Do not model new work on them. The Gazebo ground-truth markers are RViz helpers; ArUco is deprecated and not built.

## Related pages

- [IMX219 90 Hz tuning](../drone-setup/imx219_90hz_tuning.md) — kernel/device-tree work to run the sensor above 60 Hz.
- [VIO synchronization](../drone-setup/vio_synchronization.md) — hardware time-sync design (HTE). Design only; the drivers do not implement it yet.
- [H.264 video streaming](../groundstation/video-streaming.md) — viewing the camera on the ground station.
- [ROS bags](../infrastructure/ros_bags.md) — where recorded flights live and how to record.
- [Gazebo SITL validation plan](gazebo_sitl_validation_plan.md) — phased plan for qualifying perception in simulation.
