# Perception overview — workflow

**Owner:** [Ilyas Seckin](https://github.com/ilysec)

How to build the perception packages, bring them up on the drone, run them against a recorded bag, and record new data. Subsystem-specific steps live on each subsystem's workflow page.

## Build

Inside the dev container (`docker compose up -d`, then `docker compose exec ...`) or natively with ROS 2 Humble sourced:

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --packages-up-to betaflight_blackbox_driver vicon_receiver ov_msckf ekf gate_detection
source install/setup.bash
```

Notes:

- `robot_localization` is built from the fork in `src/perception/ekf/robot_localization`, not from apt. The first build takes several minutes.
- `imx219_argus_camera` only builds on aarch64 (Jetson); on x86 CMake skips it with a status message.
- `gate_detection` is a Python package. Its runtime needs `torch`, `torchvision`, `segmentation-models-pytorch` and `scipy` from `src/perception/gate_detection/requirements.txt`; `colcon` does not install them.
- `betaflight_blackbox_driver` needs `libserialport-dev` and the `msp` package. See the known issue on the [technical page](technical.md#known-issues-repository-state-september-2026) before building from a fresh clone.

## Bring up on the drone

The single entry point is `racing_bringup.launch.py`. It detects whether it runs on a Jetson and picks serial ports accordingly. The perception-relevant arguments:

| Argument | Default | Effect |
| :--- | :--- | :--- |
| `imu_source` | `blackbox` | `blackbox` (1.5 Mbaud blackbox decoder) or `msp` (MSP polling) |
| `imu_port`, `imu_baud`, `target_hz` | `/dev/ttyTHS1`, `1500000`, `150` | IMU UART and publish rate |
| `publish_raw`, `publish_rgb` | `true`, `true` | `/cam0/image_raw` (mono8) and `/cam0/image_rgb` |
| `video_width`, `video_height`, `video_fps` | `1280`, `720`, `60` | Camera mode (sensor mode 4) |
| `camera_info_url` | `file:///home/aero/proj/ETHRC-Drone-Racing/src/perception/camera_calibration/camera_info.yaml` | Intrinsics published on `/cam0/camera_info` (absolute path on the drone) |
| `view_stream`, `h264_backend` | `true`, `gstreamer_ros` | H.264 path; see [Camera → Workflow](../camera/workflow.md) |
| `use_vicon`, `vicon_subject`, `vicon_segment` | `true`, hostname, hostname | Vicon receiver and topic name |
| `use_vio` | `false` | OpenVINS with the `zephyrus_real` config |
| `use_ekf` | `false` | `ekf_node` with `rl_ekf.yaml` |
| `use_gate_detection`, `gate_camera_topic` | `false`, `/cam0/image_rgb` | `detector_node_v34` |
| `msp_bench_mode` | `false` | Disables the MSP bridge so a transmitter emulator can own the FC |

Perception only, no control, safe on the bench:

```bash
ros2 launch drone_control racing_bringup.launch.py msp_bench_mode:=true view_stream:=false
```

Everything the estimation team works on, in addition:

```bash
ros2 launch drone_control racing_bringup.launch.py msp_bench_mode:=true view_stream:=false \
  use_vio:=true use_ekf:=true
```

Expect OpenVINS to drift (see [VIO → Technical](../vio/technical.md#known-limitations)) and the EKF to reject its odometry until the `world`→`global` transform exists (see [EKF → Technical](../ekf/technical.md#required-transforms)).

## Verify

```bash
ros2 topic hz /cam0/image_raw           # ~60
ros2 topic hz /imu0                      # target_hz, default 150
ros2 topic hz /vicon/$(hostname)/$(hostname)
ros2 topic echo --once --qos-reliability best_effort /imu0
```

Sensor topics use best-effort QoS. A default `ros2 topic echo` (reliable) shows nothing; pass `--qos-reliability best_effort`.

## `perception.launch.py`

`launch/perception.launch.py` starts camera, gate detection, VIO and EKF without the controller. With default arguments it fails twice:

1. `use_gate_detection` defaults to `true` and the default checkpoint is not in the repository ([Gate detection → Workflow](../gate_detection/workflow.md)).
2. `use_ekf` defaults to `true` and `rl_ekf.launch.xml` requires a `pose1` argument that this launch does not pass ([EKF → Workflow](../ekf/workflow.md)).

Working invocation for camera + VIO on the drone:

```bash
ros2 launch launch/perception.launch.py use_gate_detection:=false use_ekf:=false use_video_stream:=false
```

## Replay a recorded flight

Both replay launches expect `ros2 bag play --clock` and set `use_sim_time:=true` on every node.

Real OpenVINS + EKF, with the static transforms the EKF needs:

```bash
# A
ros2 launch launch/perception_replay.launch.py
# B
ros2 bag play --clock <bag_dir>
# C
ros2 bag record -o out/replay /ekf/odometry /ov_msckf/odomimu /gate_detection/drone_pose /tf /tf_static
```

Vicon substituted for VIO (the configuration the EKF was tuned on):

```bash
ros2 launch launch/perception_replay_vio_subst.launch.py
ros2 bag play --clock <bag_dir>
```

`scripts/perception/replay/run_legacy_ekf_replay.sh` rebuilds `robot_localization` at the pre-fork commit, runs the same replay with `rl_ekf_legacy.yaml`, and restores the fork afterwards. Metrics and the reasoning behind the shipped filter are in `NOTES_EKF_PRODUCTION_TEST.md` at the repository root.

## Record

Recorded flights are registered in [ROS bags](../../infrastructure/ros_bags.md). For a perception bag on the drone:

```bash
ros2 bag record /cam0/image_raw /cam0/camera_info /imu0 /motor_rpm /battery \
  /vicon/$(hostname)/$(hostname) /tf /tf_static
```

`/cam0/image_raw` at 1280×720 mono8 is about 55 MB/s; `/cam0/image_rgb` three times that. Record RGB only when a colour dataset is the goal.

## Simulation

In SITL (`sitl:=true`) `racing_bringup.launch.py` does not start the IMU driver or Vicon; `state_bridge_node` reads Gazebo odometry instead. The RViz ground-truth helpers run with:

```bash
ros2 launch gazebo_ground_thruth gazebo_ground_thruth.launch.py world_frame:=world
```

The SITL stack itself is documented under [SITL Simulation](../../getting-started/sitl_simulation.md).
