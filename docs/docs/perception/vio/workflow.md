# VIO — workflow

**Owner:** [Giacomo Piatelli](https://github.com/CornelioP)

## Build

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to ov_msckf
source install/setup.bash
```

Build in Release; OpenVINS in Debug cannot keep up with 60 Hz images.

## Run

On the drone, through the bringup:

```bash
ros2 launch drone_control racing_bringup.launch.py msp_bench_mode:=true view_stream:=false use_vio:=true
```

Standalone with an explicit config (always pass `use_stereo:=false max_cameras:=1`; without them the node segfaults on startup):

```bash
ros2 launch ov_msckf subscribe.launch.py \
  config:=zephyrus_real \
  config_path:=$(ros2 pkg prefix ov_msckf)/share/ov_msckf/config/zephyrus_real/estimator_config.yaml \
  use_stereo:=false max_cameras:=1 rviz_enable:=false
```

In Gazebo, use `config:=gazebo_sim` and the matching `config_path`, and add `use_sim_time:=true`.

Against a bag:

```bash
ros2 launch launch/perception_replay.launch.py ov_config:=zephyrus_real
ros2 bag play --clock <bag_dir>
```

## Verify

```bash
ros2 topic hz /imu0 /cam0/image_raw          # both must be alive before OpenVINS initializes
ros2 topic hz /ov_msckf/odomimu               # appears ~1 s after initialization, at IMU rate
ros2 topic echo --once /ov_msckf/odomimu --field pose.pose
ros2 run rqt_image_view rqt_image_view        # /ov_msckf/trackhist shows tracked features
```

Initialization needs motion (`init_dyn_use: true`, `init_imu_thresh: 1.5`): lift and move the drone, or start bag playback from a segment with motion.

## Change the camera model

To make OpenVINS use the calibrated IMX219 instead of the Gazebo values:

```bash
cp src/perception/camera_calibration/imx219_720p_60fps_rms066px_kalibr.yaml \
   src/perception/VIO/open_vins/config/zephyrus_real/kalibr_imucam_chain.yaml
```

then add the `T_imu_cam` block (4×4, camera pose in the IMU frame) to that file from a Kalibr camera–IMU run, and set `time_offset` in `kalibr_imu_chain.yaml`. Rebuild is not needed; configs are read at start. Commit inside the submodule first, then update the pointer in the main repository.

## Troubleshooting

| Symptom | Cause | Fix |
| :--- | :--- | :--- |
| Segfault right after start | `use_stereo`/`max_cameras` not passed | `use_stereo:=false max_cameras:=1` |
| Never initializes | No motion, or IMU/camera not both arriving | Move the drone; check both `topic hz`; check `rostopic` keys in the Kalibr YAMLs |
| `parameter '...' present in ROS2 overrides but type mismatched` | Launch argument type differs from YAML | Harmless (falls back to YAML); fix the launch argument type if you meant to override |
| Estimate explodes within seconds | Wrong intrinsics/extrinsics (the current `zephyrus_real` state) | See *Change the camera model* |
| EKF ignores `/ov_msckf/odomimu` | No `world`→`global` transform | Use `perception_replay.launch.py`, or publish the static TF yourself |
| Need a backtrace | | In `subscribe.launch.py` add `prefix=['gdb -ex run -ex "bt full" -ex quit --args']` to the `run_subscribe_msckf` `Node` |

## Evaluate a trajectory

The fork's `ov_eval` has a ROS 2 `pose_to_file` recorder (`ov_eval/src/pose_to_file_ros2.cpp`) that writes `/ov_msckf/poseimu` and a ground-truth pose topic to text files for `ov_eval` error tools. See the upstream [OpenVINS evaluation docs](https://docs.openvins.com/eval-error.html) for the file format and metrics.
