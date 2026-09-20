# Gazebo SITL Perception Validation Plan

This page is the implementation contract for verifying the Boreas perception
pipeline in Gazebo before any perception-in-the-loop or real-flight test.
Rendering an image is not sufficient: the simulated camera, IMU, clocks,
frames, detector, VIO, fusion, and controller input must reproduce the real
ROS interfaces and must be scored against simulation-only ground truth.

## Current status

The existing SITL is suitable for Betaflight control testing, but it does not
yet validate the real perception pipeline:

- Gazebo publishes one `640x480 @ 30 Hz` RGB image and bridges it to
  `/cam0/image_raw`; hardware expects separate mono and RGB streams and GateNet
  defaults to `/cam0/image_rgb`.
- Simulated intrinsics, camera pose, distortion, rate, and IMU relationship do
  not match the IMX219/Boreas calibration.
- `/clock` and authoritative camera/IMU TFs are missing from the integrated
  path, and VIO is not consistently configured for simulation time.
- Gate pose estimation ignores `CameraInfo`, uses legacy geometry and dummy
  corner priors, has no reliable gate association, and does not publish a valid
  global vehicle pose/covariance.
- SITL control consumes Gazebo odometry directly. VIO, gate localization, and
  EKF output are therefore not in the closed loop.
- The current world has insufficient visual texture and gate coverage for a
  meaningful VIO/racing benchmark.

No perception result should be described as qualified until the phases below
pass in order.

### Phase-1 groundwork implemented

The first safe ingress slice is now in place: Gazebo `/clock` is bridged and
OpenVINS receives `use_sim_time:=true` when `sitl:=true` (hardware remains on
wall time).  Run the static preflight before changing any sensor bridge:

```bash
python3 scripts/simulation/validate_sitl_perception_contract.py
```

It locks the currently implemented native Gazebo camera (`30 Hz`) and IMU
(`250 Hz`) source topics and their ROS bridge types. It does **not** certify
calibration, mono/RGB conversion, camera frame IDs, or runtime timing. Those
remain owned by the forthcoming sensor adapter; the present bridge still feeds
the legacy production names directly and must not be used to claim perception
qualification.

## Target ROS contract

| Item | Contract |
|---|---|
| Global course frame | `map`, ENU |
| Continuous estimator frame | `odom`, ENU |
| Vehicle frame | `base_link`, FLU |
| IMU frame | `imu_link` |
| Camera frame | `cam0_optical`, REP-103 optical convention |
| Mono image | `/cam0/image_raw`, `mono8` |
| RGB image | `/cam0/image_rgb`, `rgb8` |
| Camera calibration | `/cam0/camera_info`, same source stamp/frame as images |
| IMU | `/imu0`, SI units and the same clock epoch as images |
| VIO | `/ov_msckf/odomimu`, `odom -> base_link` |
| Relative gate observation | `/gate_detection/observation` |
| Global gate-derived fix | `/gate_detection/drone_pose`, frame `map` |
| Fused estimate | `/state_estimate/odometry` |
| Simulation-only truth | `/sim/ground_truth/*` |

The TF tree is `map -> odom -> base_link`, with calibrated static transforms
from `base_link` to `imu_link` and `cam0_optical`. Simulation truth must never
be subscribed by detector, VIO, EKF, or the controller in a qualification run.

## Phase 0 — Freeze the physical sensor contract

Create authoritative Boreas files under `config/perception/` for camera
intrinsics, distortion, resolution/rate/encoding, IMU axes/noise/rate,
camera-to-IMU/base extrinsics, and race-gate/course geometry.

Required physical work:

1. Run the existing Kalibr camera-IMU calibration workflow.
2. Record a six-axis stationary/motion bag to verify Blackbox IMU signs.
3. Measure the real gate inner/outer dimensions and camera mounting pose.

Acceptance: schema tests pass; transforms are proper rotations; calibration
matches the selected resolution; recorded IMU signs agree with the documented
FLU/optical frame contract. Do not substitute identity transforms.

## Phase 1 — Sensor-equivalent Gazebo ingress

Primary files:

- `sim/gazebo/models/drone_sitl/model.sdf`
- `src/utils/gazebo_ros_bridges/config/gz_bridges.yaml`
- new `src/perception/sim_sensor_adapter/`
- `src/drone_control/launch/racing_bringup.launch.py`

Bridge native Gazebo streams to private `/sim/...` topics. A sensor adapter
must publish the production mono/RGB/CameraInfo/IMU topics with identical
source stamps, calibrated frame IDs and configurable deterministic profiles:
`ideal`, `nominal`, and `stress`. The adapter owns mono conversion, latency,
jitter, frame drops, noise, blur, and exposure effects that Gazebo cannot
represent reliably. Bridge `/clock` and set `use_sim_time=true` throughout the
SITL perception graph.

Acceptance over 60 seconds:

- configured camera rate within `+-2 Hz`, IMU rate within its contract;
- image and CameraInfo stamps identical, monotonic, and from simulation time;
- correct encodings, dimensions, stride, frame IDs, and TF connectivity;
- less than 1% unexpected drops and no unbounded queues;
- headless real-time factor at least `0.9` at the selected qualification mode.

Benchmark `1280x720 @ 60 Hz` before making it mandatory on every developer
host; retain a clearly labelled fast mode for CI.

## Phase 2 — Racing scene and scoring oracle

Add `sim/gazebo/worlds/sitl_perception_validation.sdf`, generated or validated
against an authoritative race-course YAML. It must contain multiple uniquely
named gates, measured gate geometry, feature-rich licensed textures,
asymmetric landmarks, lighting presets and controlled occluders.

Add scoring-only publishers for drone odometry, gate poses, visible gate ID,
projected corners and optional masks under `/sim/ground_truth/`. These topics
must be isolated from the operational perception graph.

Acceptance:

- course YAML and SDF poses agree within `1 mm` and `0.01 deg`;
- a calibration fixture projects within `2 px` of the analytic oracle;
- gates remain measurable from `2--10 m` and representative view angles;
- no missing texture/model warnings in headless rendering.

## Phase 3 — Correct gate-detection semantics

Refactor gate detection into segmentation, relative-pose estimation and map
localization. It must consume synchronized `CameraInfo`, undo resizing or
letterboxing, support the calibrated distortion model, load shared gate
geometry, and reject PnP solutions using cheirality and reprojection error.

The relative output is `T_cam_gate` with gate ID, confidence, corners,
reprojection error and covariance. A global fix is legal only with a valid gate
association:

```text
T_map_base = T_map_gate * inverse(T_cam_gate) * T_cam_base
```

Acceptance in the nominal profile: recall at least 95%, false positives below
1%, corner RMSE below 3 px, relative position error below 0.15 m, attitude
error below 3 degrees, valid normalized quaternions, calibrated covariance and
Jetson p95 latency below 50 ms. Re-evaluate thresholds against held-out real
data rather than tuning them per simulated scene.

## Phase 4 — VIO, time and frame qualification

Fix OpenVINS launch propagation of `use_sim_time`. Add a deterministic
`boreas_gazebo` configuration from the simulated authoritative calibration and
a `boreas_real` configuration from Kalibr. Camera orientation must be identical
on the RGB and mono branches.

Exercise stationary, translation, yaw, figure-eight and racing-speed scripted
trajectories while flight control still uses Gazebo truth.

Acceptance over 60 seconds: no resets, NaNs or time regressions; VIO ATE RMSE
below 0.15 m, drift below 1% of distance, RPE at 1 second below
`0.10 m / 2 deg`, velocity RMSE below 0.3 m/s and no loss longer than 0.5 s.

## Phase 5 — Fusion in shadow mode

Add SITL-specific EKF configurations for `vio_only`, `vio_gate`, and
`vicon_gt`. Fuse validated VIO and gate fixes using simulation time and
Mahalanobis rejection. Keep control on ground truth while scoring the fused
estimate; inject known outliers and dropouts.

Acceptance: finite continuous output; no ground-truth subscription by the
operational graph; nominal fusion does not regress VIO; gate updates reduce
drift; injected outliers are rejected; correction jumps remain below
`0.25 m / 5 deg`.

## Phase 6 — Perception-in-the-loop SITL

Add an explicit `state_source:=gazebo_gt|perception` launch argument. The
default remains `gazebo_gt`. Qualification mode must route only
`/state_estimate/odometry` into the controller and fail launch if a silent
ground-truth fallback exists. Add estimator-age/covariance watchdogs and a
disarm/termination response.

Implement `scripts/simulation/validate_sitl_perception.py` with bag and JSON
artifacts. Run `ideal|nominal|stress` profiles across multiple seeds for hover,
cardinal motion and a short gate course.

Initial closed-loop gate: ideal and nominal pass 10/10 runs; stress passes at
least 9/10; zero collisions, NaNs or estimator-age events above 100 ms; at
least 95% gates completed over 30 seeds; lap time within 10% of the identical
ground-truth-state baseline.

## Phase 7 — Sim-to-real evidence

Add a perception bag preset containing raw/RGB images, CameraInfo, IMU, Vicon
truth, VIO, detector internals, EKF, commands and system load. Replay real bags
through the same detector/VIO/EKF launch and compare brightness, blur, feature
count, confidence, latency, IMU PSD, bias and dropout distributions.

Tune named simulation sensor profiles from real measurements. Train using
randomized Gazebo plus real data with flight-level held-out splits. First run
the real system in shadow mode while Vicon controls, then use bounded low-speed
perception control with geofence and kill switch before expanding the envelope.

## Delivery order

1. Sensor/frame/time contracts, adapter, validation world and oracle evaluator.
2. Gate detector semantic corrections and unit/integration tests.
3. VIO calibration, clock handling and trajectory benchmark.
4. EKF shadow fusion and outlier tests.
5. Explicit perception state source and closed-loop qualification matrix.
6. Domain randomization and held-out real-bag benchmark.

Each pull request must include machine-readable metrics. Pixel-perfect golden
images are not portable across GPUs; use geometric and statistical tolerances
and pin the Gazebo/Ogre2 version used for qualification.
