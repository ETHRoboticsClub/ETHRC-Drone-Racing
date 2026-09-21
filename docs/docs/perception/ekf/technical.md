# EKF — technical

**Owner:** [Oliver Calvet](https://github.com/oliomap)

The EKF fuses relative odometry (OpenVINS) with sparse absolute poses (gate detection) into one state estimate. It is `robot_localization`'s `ekf_node`, built from a fork in which the stock 15-state Euler filter was replaced by a **16-state quaternion filter with quadrotor dynamics** in the predict step. Package `src/perception/ekf/ekf` holds our configs, launch files and tests; the filter itself is the submodule `src/perception/ekf/robot_localization` (`oliomap/robot_localization_drone`, pinned at `484337f`).

Status: validated in replay only. On the 2026-04-16 bag with Vicon substituted for VIO, the replay-tuned variant reached 5.7 cm position RMS and 1.7° orientation RMS against 7.8 cm / 2.4° for the stock filter; the shipped variant shares its orientation and horizontal behaviour but its Z figure has not been re-measured (`NOTES_EKF_PRODUCTION_TEST.md`). Not used by the controller in flight.

## State

```
index:   0  1  2   3   4   5   6   7   8   9   10     11      12    13  14  15
state:  [X, Y, Z,  Qw, Qx, Qy, Qz, Vx, Vy, Vz, Vroll, Vpitch, Vyaw, Ax, Ay, Az]
         world ENU  body→world quat   body velocity  body rates       body accel
```

Compared with stock `robot_localization`, orientation occupies four slots instead of three and every later index is shifted by one. **Every `*_config` array in the YAML therefore has 16 booleans**; a 15-element array silently maps sensors to the wrong states.

## Predict step

Constants: `g = 9.81 m/s²`, `τ_w = 0.1 s`. With `R` the body→world rotation of `q`:

| Quantity | Model |
| :--- | :--- |
| Orientation | `q ← normalize(q + ½ Ξ(q) ω dt)` |
| Position | `p ← p + R v dt + ½ R a dt²` |
| Velocity | `v ← v + a dt` |
| Body acceleration (set before propagation, from attitude under the hover-thrust assumption `T ≈ mg`) | `Ax = +2g (Qw Qy − Qx Qz)`, `Ay = −2g (Qw Qx + Qy Qz)`, `Az = +2g (Qx² + Qy²)` |
| Body rates | `ω ← ω (1 − dt/τ_w)` |

The Jacobian is the transfer matrix plus the analytical ∂q/∂q, ∂q/∂ω and ∂(v,a)/∂q terms (`ekf.cpp`, `Ekf::predictDt`). The Euler `1/cos(pitch)` singularity of the stock filter is gone; the quaternion is re-normalized after every predict and correct.

!!! note "Two variants exist in the fork's history"
    Commit `484337f` (pinned) is the full physics above. Tag `replay-z-kludge` (`2308a32`) pins `Az = 0` and zeroes the Z column of the horizontal velocity/acceleration coupling; it scored better on the Vicon-substituted replay but is not physically consistent and is not shipped. The doc-comment above `predictDt` in `ekf.cpp` still describes the `Az = 0` variant — the code below it is authoritative.

Default process noise (diagonal, `filter_base.cpp`): position 0.05/0.05/0.06, quaternion 0.03 on all four components (isotropic on S³ — leaving `Qz` at zero was a real bug, `78395d3`), velocity 0.025/0.025/0.04, rates 0.01/0.01/0.02, acceleration 0.01/0.01/0.015. The quaternion is seeded to identity at reset. `rl_ekf.yaml` keeps the matrices commented out and uses these defaults.

## Inputs (`config/rl_ekf.yaml`)

| Input | Topic | Type | Fused states |
| :--- | :--- | :--- | :--- |
| `odom0` | `/ov_msckf/odomimu` | `nav_msgs/Odometry` | position, quaternion, body velocity, body rates (not acceleration) |
| `pose0` | `/gate_detection/drone_pose` | `geometry_msgs/PoseWithCovarianceStamped` | position, quaternion |
| `pose1` | `/vicon/zephyrus/zephyrus` (via launch argument) | | **Commented out** — the entry still has the 15-element layout and must be widened before use |
| `imu0` | `/imu/data` | | Commented out |
| `control0` | `cmd_vel` → `/control/command` | `geometry_msgs/TwistStamped` | `use_control: false` |

Other settings: `frequency: 100`, `sensor_timeout: 0.1`, `predict_to_current_time: true`, `smooth_lagged_data: true` with `history_length: 0.5` (a measurement older than the kept history cannot be fused), queue size 10 per input, no Mahalanobis rejection thresholds set, `print_diagnostics: true`.

Frames: `map_frame: world`, `world_frame: world`, `odom_frame: odom`, `base_link_frame: base_link`. Because `world_frame == map_frame`, absolute poses are fused directly and the published transform is `odom`→`base_link` with `odom` coinciding with `world`.

## Outputs

| Topic | Type | Content |
| :--- | :--- | :--- |
| `/ekf/odometry` (remapped from `odometry/filtered`) | `nav_msgs/Odometry` | `pose` in `world`; `twist` in **body frame**; 6×6 pose covariance with the orientation block projected from the 4×4 quaternion covariance by a first-order Jacobian |
| `/accel/filtered` | `geometry_msgs/AccelWithCovarianceStamped` | Body-frame acceleration states |
| `/tf` | `odom` → `base_link` | 100 Hz |

## Required transforms

`robot_localization` transforms every measurement into `world_frame` (poses) or `base_link_frame` (twists) using TF. With the production inputs this means:

| Transform | Why | Published by |
| :--- | :--- | :--- |
| `world` → `global` | `/ov_msckf/odomimu` has `frame_id: global` | only `launch/perception_replay.launch.py` |
| `base_link` → `imu` | its `child_frame_id` is `imu` | only the replay launches |
| `world` → `map` | if Vicon is fused or compared | only the replay launches |

`racing_bringup.launch.py` publishes none of them, so with `use_ekf:=true use_vio:=true` on the drone the EKF silently ignores the VIO odometry — the transform-failure warning is commented out in `ros_filter_utilities.cpp` — and only fuses gate poses, whose `frame_id` is already `world`.

## What the fork changes

| File | Change |
| :--- | :--- |
| `include/robot_localization/filter_common.hpp` | `STATE_SIZE 16`, `StateMemberQw..Qz` at 3–6, backward-compatible aliases (`StateMemberRoll = StateMemberQw`, …). `ORIENTATION_SIZE` stays 3 because it sizes ROS covariance blocks |
| `src/ekf.cpp` | `predict()` rewritten as above; `correct()` drops the Euler innovation wrapping (quaternion sign consistency is handled when the measurement is prepared) |
| `src/filter_base.cpp` | `wrapStateAngles()` normalizes the quaternion; 16×16 defaults; identity seed |
| `src/ros_filter.cpp` | Reads quaternions from messages (with hemisphere flip), converts RPY covariance with `dQuatDRPY()`, writes the quaternion straight into the output |
| `src/ros_filter_utilities.cpp` | `stateToTF` / `TFtoState` without the RPY round trip |
| `test/test_drone_ekf.cpp` | Five gtests on the predict step (hover, norm, gravity at hover, tilt coupling, rate decay) |

Only `ekf_node` is supported. `ukf_node` compiles through the aliases but interprets quaternion slots as Euler angles.

## Test infrastructure (`ekf` package)

`mock_publisher` generates synthetic sensor streams so the filter can be exercised without a simulator: `/vio/odometry` (`nav_msgs/Odometry`, `vio_rate_hz` 100), `/vision/pose` (`PoseWithCovarianceStamped`, `vision_rate_hz` 60), `/control/command` (`TwistStamped`, 100 Hz), and exits after `duration_sec`. Scenarios: `rest`, `const_vel`, `turn`, `accel`, `sprint`, `slalom`, `pitch_dive`, `dropout`. `config/test_ekf.yaml` and `config/hard_test_ekf.yaml` point the filter at those topics; `launch/scenario_record.launch.py` and `hard_scenario_record.launch.py` run filter + mock + `ros2 bag record` + CSV export into `tests/<scenario>_<bagname>/`. Notebooks in `plots/` compare runs.

## Files

| Path | Purpose |
| :--- | :--- |
| `src/perception/ekf/ekf/config/rl_ekf.yaml` | Production config |
| `src/perception/ekf/ekf/config/rl_ekf_legacy.yaml` | 15-state config for A/B replays against upstream `robot_localization` |
| `src/perception/ekf/ekf/config/test_ekf.yaml`, `hard_test_ekf.yaml` | Mock-publisher configs |
| `src/perception/ekf/ekf/launch/rl_ekf.launch.xml` | Node launch; needs `pose1` |
| `src/perception/ekf/ekf/launch/*_scenario_record.launch.py` | Scenario runs |
| `src/perception/ekf/ekf/src/nodes/mock_publisher.cpp` | Synthetic sensors |
| `src/perception/ekf/ekf/test/test_drone_ekf.cpp` | gtests (need `robot_localization` built with testing) |
| `src/perception/ekf/ekf/SETUP.md` | Longer integration guide from the refactor; its `/aruco/drone_pose` references are outdated |
| `src/perception/ekf/robot_localization/src/ekf.cpp` | The filter |
| `NOTES_EKF_PRODUCTION_TEST.md` (repo root) | Replay metrics, kludge analysis, triage levers |

## Known limitations

- Never run in closed loop on hardware; the controller reads Vicon through `state_bridge_node`.
- Depends on OpenVINS, which is not calibrated for the real camera; the validation used Vicon as a stand-in for VIO.
- `rl_ekf.launch.xml` does not start without a `pose1` argument (see [EKF → Workflow](workflow.md)).
- No rejection thresholds: a bad gate pose is fused with whatever covariance the detector attaches (0.05 m, 5° by default).
- The IMU is not fused; `/imu0` would need `base_link`→`imu_link` and a 16-element `imu0_config`.
