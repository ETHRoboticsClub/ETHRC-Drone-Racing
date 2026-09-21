# EKF — workflow

**Owner:** [Oliver Calvet](https://github.com/oliomap)

## Build

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --packages-up-to ekf
source install/setup.bash
```

This builds the `robot_localization` fork first (several minutes, heavy Eigen templates). Afterwards rebuild only what changed:

```bash
colcon build --symlink-install --packages-select ekf                  # configs, launch, mock publisher
colcon build --symlink-install --packages-select robot_localization   # filter source
```

If `colcon` complains about a symbolic link that "is a directory" under `build/robot_localization/ament_cmake_python`, delete that directory and rebuild with `--allow-overriding robot_localization`.

## Run

`rl_ekf.launch.xml` remaps a Vicon topic to `$(var pose1)` and does not declare a default, so **`pose1` is mandatory** even though the Vicon input is disabled in the YAML:

```bash
ros2 launch ekf rl_ekf.launch.xml pose1:=/vicon/$(hostname)/$(hostname)
```

Without it: `Caught exception in launch ... launch configuration 'pose1' does not exist`.

Through the bringup (which passes `pose1`):

```bash
ros2 launch drone_control racing_bringup.launch.py msp_bench_mode:=true view_stream:=false \
  use_vio:=true use_ekf:=true
```

Remember that on the drone nothing publishes `world`→`global`, so VIO odometry is not fused this way; see [EKF → Technical → Required transforms](technical.md#required-transforms). Add it by hand for a bench test:

```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world global
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link imu
```

Against a bag (transforms included):

```bash
ros2 launch launch/perception_replay.launch.py            # real OpenVINS
ros2 launch launch/perception_replay_vio_subst.launch.py  # Vicon as VIO stand-in
ros2 bag play --clock <bag_dir>
```

## Verify

```bash
ros2 topic hz /ekf/odometry                     # ~100 Hz once the first measurement arrived
ros2 topic hz /ov_msckf/odomimu /gate_detection/drone_pose
ros2 topic echo --once /ekf/odometry --field pose.pose.orientation   # unit norm; identity before init
ros2 run tf2_tools view_frames                  # odom -> base_link present
```

With `print_diagnostics: true` the node publishes `/diagnostics` with per-input warnings (bad covariances, stale stamps). For a full trace set `debug: true` in the YAML (writes `/tmp/ekf_debug.txt`, large).

## Synthetic scenarios

Run the filter against `mock_publisher` and record everything into `tests/<scenario>_<bagname>/` in the current directory:

```bash
ros2 launch ekf scenario_record.launch.py scenario:=turn duration:=30.0        # rest, const_vel, turn, accel
ros2 launch ekf hard_scenario_record.launch.py scenario:=sprint duration:=60.0 # sprint, slalom, pitch_dive, dropout
```

Outputs: `rosbag/` (all topics), `scenario_data.csv` (`/ekf/odometry`), `accel_data.csv` (`/accel/filtered`). The hard variant disables VIO orientation and runs vision at 5 Hz. Analysis notebooks: `plots/compare_ekf.ipynb`, `plots/hard_ekf.ipynb`.

Unit tests of the predict step:

```bash
colcon build --packages-select ekf --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select ekf && colcon test-result --verbose
```

## Compare with the legacy filter

```bash
EKF_CONFIG_FILE=rl_ekf_legacy.yaml LAUNCH_FILE=launch/perception_replay_vio_subst.launch.py \
  ./scripts/perception/replay/run_legacy_ekf_replay.sh
```

The script checks out the pre-fork `robot_localization` commit, rebuilds, runs the replay, and restores the fork. Run it inside the dev container.

## Change the submodule

1. `cd src/perception/ekf/robot_localization && git checkout <branch>` — the submodule is in detached HEAD by default.
2. Commit inside the submodule, adding files by path (never `git add .` there).
3. In the main repository, commit the updated submodule pointer.

## Troubleshooting

| Symptom | Cause | Fix |
| :--- | :--- | :--- |
| `launch configuration 'pose1' does not exist` | Launch file requires it | Pass `pose1:=<any topic>` |
| 0 Hz on `/ekf/odometry` | No measurement received yet | `topic hz` the inputs; the filter starts on the first one |
| `/ov_msckf/odomimu` alive but the estimate does not follow it, no warning | Missing static TF (the failure is silent) | Publish `world`→`global` (and `base_link`→`imu`) |
| Orientation stays identity | Orientation indices disabled, or input quaternion is zero | Check `odom0_config[3:7]`; echo the input orientation |
| Sensor data mapped to wrong states | 15-element `*_config` | Count to 16 |
| Position jumps on gate detections | Gate pose in another frame or wrong gate id | See [Gate detection → Technical](../gate_detection/technical.md#frames) |
| Old measurements dropped (`history` warnings) | Latency > `history_length` | Increase `history_length` in the YAML |
