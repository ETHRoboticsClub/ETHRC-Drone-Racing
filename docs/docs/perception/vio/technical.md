# VIO — technical

**Owner:** [Giacomo Piatelli](https://github.com/CornelioP)

Visual-inertial odometry estimates the drone's pose and velocity from the camera and the IMU alone, without external infrastructure. We use [OpenVINS](https://docs.openvins.com), a filter-based (MSCKF) estimator, as a fork in `src/perception/VIO/open_vins` (submodule `ETHRoboticsClub/open_vins_RETH`, pinned at `735d64e`).

Status: runs, publishes, and drifts by kilometres on real flights because it has never been configured with the real camera's intrinsics or a camera–IMU extrinsic. See [Known limitations](#known-limitations) before trusting any number it produces.

## What OpenVINS estimates

A sliding window of recent IMU poses (`max_clones`) plus a small set of long-lived landmarks (`max_slam`). Between images the IMU is integrated; on each image, tracked features constrain the window without adding every landmark to the state. The state also carries IMU biases and, when enabled, the camera intrinsics, the camera–IMU extrinsic and their time offset, all refined online.

## ROS interface

Node `run_subscribe_msckf` in namespace `ov_msckf`, launched by `ov_msckf/launch/subscribe.launch.py`.

Subscriptions (topic names come from the `rostopic` keys of the Kalibr YAMLs in the config directory):

| Topic | Type | QoS (fork) |
| :--- | :--- | :--- |
| `/imu0` | `sensor_msgs/Imu` | best-effort, depth 2000 |
| `/cam0/image_raw` | `sensor_msgs/Image` | best-effort, depth 50 |

Publications, all under `/ov_msckf/`:

| Topic | Type | Frames | When |
| :--- | :--- | :--- | :--- |
| `odomimu` | `nav_msgs/Odometry` | `global` → `imu` | every IMU sample once initialized for ≥ 1 s, only while someone subscribes; the EKF's `odom0` input |
| `poseimu` | `geometry_msgs/PoseWithCovarianceStamped` | `global` | per image |
| `pathimu` | `nav_msgs/Path` | `global` | per image |
| `points_msckf`, `points_slam` | `sensor_msgs/PointCloud2` | `global` | per image |
| `trackhist` | `sensor_msgs/Image` | `cam0` | per image, feature overlay |
| `/tf` | `global` → `imu`, `imu` → `cam0` | | per image |

Launch arguments that matter: `config` (config directory name), `config_path` (explicit path to `estimator_config.yaml`; overrides `config`), `use_stereo` and `max_cameras` (must be `false` / `1` for our mono setup, otherwise the node segfaults), `use_sim_time`, `rviz_enable`, `verbosity`.

## Configuration

Each config directory holds three files:

| File | Content |
| :--- | :--- |
| `estimator_config.yaml` | Filter, tracker and initialization settings |
| `kalibr_imu_chain.yaml` | IMU noise densities, random walks, `update_rate`, topic |
| `kalibr_imucam_chain.yaml` | Camera model, intrinsics, distortion, resolution, `T_imu_cam`, topic |

Our directories under `open_vins/config/` (installed to `share/ov_msckf/config/`):

| Directory | Meant for | Camera in the YAML |
| :--- | :--- | :--- |
| `gazebo_sim` | Gazebo x500 camera | 640×480, `radtan` zero distortion, f = 205.47, identity `T_imu_cam` |
| `zephyrus_real` | Real drone, selected by `racing_bringup` and `perception.launch.py` | **Identical to `gazebo_sim`** except `init_dyn_use: true` |

The remaining directories (`euroc_mav`, `tum_vi`, `uzhfpv_*`, `rs_t265`, …) are upstream dataset configs and useful as templates.

Settings in `zephyrus_real/estimator_config.yaml` that define how it behaves:

| Setting | Value | Effect |
| :--- | :--- | :--- |
| `use_stereo` / `max_cameras` | false / 1 | Monocular |
| `calib_cam_extrinsics`, `calib_cam_intrinsics`, `calib_cam_timeoffset` | true | Online refinement from the (wrong) priors |
| `max_clones`, `max_slam` | 11, 50 | Window size, landmark count |
| `use_klt`, `num_pts`, `track_frequency` | true, 200, 21 Hz | KLT tracking at 21 of the 60 camera frames per second |
| `init_dyn_use` | true | Dynamic initialization — required because real bags have no static phase at the start |
| `init_imu_thresh` | 1.5 | Accelerometer variance that counts as motion |
| `gravity_mag` | 9.81 | |
| IMU `update_rate` | 200 | The blackbox driver publishes at `target_hz` (150 by default) |

The correct camera values for the IMX219 exist in `src/perception/camera_calibration/imx219_720p_60fps_rms066px_kalibr.yaml` (1280×720, `equidistant`, f ≈ 559) and only need to be copied into `zephyrus_real/kalibr_imucam_chain.yaml`; the camera–IMU extrinsic and time offset still have to be calibrated ([Calibration → Workflow](../calibration/workflow.md#camera-imu-calibration-kalibr)).

## What the fork changes

Relative to upstream OpenVINS (`d0075dd`):

- Adds the `gazebo_sim` and `zephyrus_real` config directories.
- `ROS2Visualizer.cpp`: best-effort subscriptions with deep queues (IMU 2000, camera 50) so bag playback and best-effort drivers work.
- `subscribe.launch.py`: rewritten to take `config_path` and to accept our argument set.
- `ov_eval`: ROS 2 port of `pose_to_file` and a `RecorderROS2` helper for trajectory evaluation.
- `opencv_yaml_parse.h`: a ROS parameter override with the wrong type now falls back to the YAML value instead of silently keeping the default.

## Files

| Path | Purpose |
| :--- | :--- |
| `src/perception/VIO/open_vins/config/zephyrus_real/` | Config used on hardware |
| `src/perception/VIO/open_vins/ov_msckf/launch/subscribe.launch.py` | Launch file |
| `src/perception/VIO/README.md` | Upstream-style topic list (older copy of this page) |
| `launch/perception_replay.launch.py` | Replays a bag through OpenVINS + EKF with the static TFs the EKF needs |

## Known limitations

- Camera intrinsics, resolution and distortion model in `zephyrus_real` are the Gazebo camera's, not the IMX219's. Feature triangulation is therefore wrong from the first frame; online intrinsic calibration cannot recover a 2× focal-length error.
- `T_imu_cam` is identity. The real camera is mounted on the nose, tilted, and (on the NITROS path) its mono image is upside-down relative to the flipped RGB image.
- No common clock between camera and IMU; the time offset is estimated online from wrong priors.
- Output frame `global` is not connected to `world` or `map` by any transform in the bringup.
- Replays on the 2026-04-16 flight bag (`data_20260416`) drift by kilometres; this is why the EKF was tuned with Vicon substituted for VIO.
