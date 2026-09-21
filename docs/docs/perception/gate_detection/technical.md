# Gate detection — technical

**Owner:** [Mahbod Tajdini](https://github.com/Ace3Z)

Gate detection turns a camera image into an absolute pose of the drone: find the gate, locate its four inner corners in the image, solve PnP against the known gate geometry, and chain the result through the gate's known position in the course. The output is the EKF's `pose0` input. Package: `src/perception/gate_detection` (Python, `ament_python`).

Status: experimental. The node that the launch file starts needs a model checkpoint that is not in the repository, and its `camera_info` subscription cannot connect to any camera driver in this repository. Neither has been fixed because the detector is not on the flight path.

## Three generations in one package

| Generation | Code | Method | Where it is used |
| :--- | :--- | :--- | :--- |
| v1 GateNet + QuAdGate | `models/gatenet.py`, `models/quadgate.py`, `checkpoints/best_gatenet.pth`, `scripts/inference_standalone.py`, `utils/*` (dataset tools) | U-Net segmentation, LSD line intersections for sub-pixel corners | Offline scripts only; no node imports it |
| AIGP colour detector | `detector_node.py` (executable `detector_node`) | Orange HSV segmentation + contour quad + PnP against the AIGP track YAML; hard-coded 640×360 intrinsics | `aigp_bringup.launch.py`; feeds `rl/aigp_state_estimator_node`, not the EKF |
| **v3.4 hybrid** | `detector_node_v34.py` (executable `detector_node_v34`), `v3_4/` | DeepLabV3+ with segmentation, keypoint and next-gate heads + hybrid tracker + IPPE PnP | `launch/gate_detection.launch.py` → `racing_bringup`, `perception.launch.py` |

The rest of this page describes v3.4. The AIGP detector is summarized at the end.

## v3.4 pipeline (`detector_node_v34.py`)

1. **Predict.** `GatePredictor` (`v3_4/inference/predictor.py`) resizes the BGR frame to `img_size` (384×384) and runs `smp.DeepLabV3Plus` (ResNet-34 encoder) with three heads: gate segmentation mask, a 4-channel keypoint heatmap in the order inner-TL, TR, BR, BL, and a "next gate" mask. fp16 autocast on CUDA. Checkpoints store their `ModelConfig`; the node refuses checkpoints without the keypoint head and warns without the next-gate head.
2. **Select and track.** `HybridSegNextGateTracker` (`v3_4/inference/hybrid_tracker.py`) takes connected components of the segmentation mask, picks the one with the largest overlap with the next-gate mask (fallback: largest component, logged), extracts one quad from the keypoint peaks inside its bounding box, and holds the track across up to `max_fail_streak` (3) bad frames. A `handoff_area_frac` of 0.15 releases the lock when the gate fills the image.
3. **Rescale and undistort.** Corners go from 384×384 to native pixels with a half-pixel-correct mapping, then through `cv2.fisheye.undistortPoints` (`equidistant`/`fisheye`) or `cv2.undistortPoints` (`plumb_bob`, `rational_polynomial`, empty), using the latched `camera_info`.
4. **PnP.** `cv2.solvePnP(..., SOLVEPNP_IPPE)` with the planar inner-corner model in the gate frame: half-width 0.75 m, top edge at z = 1.70 m, bottom edge at z = 0.35 m (a 1.50 × 1.35 m opening; inherited from the legacy node, not confirmed against a drawing).
5. **Chain to world.** `world_T_body = world_T_gate · gate_T_cam · cam_T_body`. `cam_T_body` is a fixed FPV stub (optical z = body +x, zero translation). `world_T_gate` comes from the parkour map and a gate-id selector: bootstrap to `start_gate_id`, then predict the gate position from the last published body pose and pick the nearest map gate; a prior older than 0.5 s forces a re-bootstrap; a nearest-gate distance over 10 m is logged as suspect.
6. **Publish** `PoseWithCovarianceStamped` in frame `world`, stamp copied from the image, diagonal covariance from `pose_position_std_m` and `pose_rotation_std_deg`.

Every fallback is logged with a `[WARN]` prefix, once or rate-limited; nothing fails silently.

## Interface

| Direction | Topic (parameter) | Type | QoS |
| :--- | :--- | :--- | :--- |
| in | `/cam0/image_rgb` (`camera_topic`) | `sensor_msgs/Image` (any encoding `cv_bridge` can convert to BGR) | best-effort, depth 1 |
| in | `/cam0/camera_info` (`camera_info_topic`) | `sensor_msgs/CameraInfo`, latched on first message | **reliable, transient-local** |
| out | `/gate_detection/drone_pose` (`output_topic`) | `geometry_msgs/PoseWithCovarianceStamped`, frame `world` | reliable, depth 10 |

| Parameter | Default | Meaning |
| :--- | :--- | :--- |
| `checkpoint` | `share/gate_detection/checkpoints/best_deeplabv3plus_keypoint_multi_balanced_v3_nextgate_seed42.pth` | v3.4 weights. Not in the repository |
| `device` | auto (`cuda` > `mps` > `cpu`) | |
| `img_size` | 384 | Network input size |
| `max_rate_hz` | 30 | Frames above this rate are skipped |
| `parkour_map_path` | `share/gate_detection/config/parkour_map.yaml` | Empty string disables the map (identity `world_T_gate`, logged) |
| `start_gate_id` | 0 | Gate the drone starts in front of. A wrong value is a stable fixed point — the selector cannot recover on its own |
| `pose_position_std_m`, `pose_rotation_std_deg` | 0.05, 5.0 | Published covariance |

## Frames

| Frame | Definition |
| :--- | :--- |
| Gate-local | Origin on the floor under the gate centre, +x through the gate (flight direction), +y left, +z up. Corner model has x = 0 |
| Camera | OpenCV optical: x right, y down, z forward |
| Body | REP-103 FLU; `cam_T_body` assumes the camera looks along body +x with no offset or tilt |
| `world` | Output frame. Its origin and axes are whatever the parkour map says |

`config/parkour_map.yaml` declares `world_frame: NED` and lists the six AIGP course gates (x from −23 m to −159 m, z positive downward) with identity orientation. The node composes NED gate positions with FLU/ENU-style local geometry and publishes the result as `world`, which the EKF treats as ENU. Until a map for the lab exists and the frame conventions are reconciled, the published pose is only self-consistent for a single gate.

## AIGP colour detector (`detector_node.py`)

Subscribes to `/cam0/image_rgb`, `/aigp/active_gate_index` (`Int32`) and `/drone_state_estimated` (`Float64MultiArray`); publishes `/aigp_state_estimator/set_pose`. Segments orange in HSV, scores contours for squareness and temporal continuity, refines corners with `cornerSubPix`, solves PnP with the 2.7 m outer-gate model and the simulator's 640×360 pinhole (`f = 320`), and rejects solutions inconsistent with the estimated state (`max_position_innovation_m` 3 m) or with reprojection error above `max_reprojection_error_px` (2 px). Camera tilt `camera_up_tilt_deg` defaults to 17.3° (calibrated from ground truth; the simulator documentation says 20°). Track geometry comes from `rl/config/aigp_competition_track.yaml`. Tests: `test/test_detector_math.py`.

## Files

| Path | Purpose |
| :--- | :--- |
| `src/perception/gate_detection/gate_detection/detector_node_v34.py` | v3.4 ROS node (the module docstring is the design spec) |
| `src/perception/gate_detection/gate_detection/v3_4/` | Predictor, trackers, model definitions ported from `DroneRacing_GateDetection` commit `14d21f3` |
| `src/perception/gate_detection/config/parkour_map.yaml` | AIGP gate positions (NED) |
| `src/perception/gate_detection/launch/gate_detection.launch.py` | Starts `detector_node_v34` |
| `src/perception/gate_detection/checkpoints/` | Only `best_gatenet.pth` (v1) is checked in |
| `src/perception/gate_detection/tests/test_a11_gate_id_selection.py` | Gate-id selector desk simulation (pure numpy) |
| `src/perception/gate_detection/requirements.txt` | Pinned Python deps (`torch 2.11`, `segmentation-models-pytorch 0.5.0`, …) |
| `src/perception/gate_detection/gate_detection/detector_node.py` | AIGP colour detector |

## Known limitations

- The default checkpoint is absent; the node raises `FileNotFoundError` at start.
- `camera_info` is requested with transient-local durability. Both in-repo camera drivers publish volatile `camera_info`, which DDS will not match, so the node drops every frame with `received image before camera_info`. Whether the Isaac ROS Argus node publishes transient-local was not verified.
- Frame mismatch between the NED parkour map and the ENU EKF (see [Frames](#frames)).
- `cam_T_body` and the gate dimensions are placeholders.
- No temporal filtering of the published pose beyond the tracker's hold logic; covariance is constant, not derived from PnP residuals.
- Inference speed on the Jetson has not been measured; no TensorRT export exists.
