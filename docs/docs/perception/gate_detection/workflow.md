# Gate detection — workflow

**Owner:** [Mahbod Tajdini](https://github.com/Ace3Z)

## Prerequisites

Python dependencies are not installed by `colcon`:

```bash
pip install -r src/perception/gate_detection/requirements.txt   # torch, torchvision, smp, scipy, opencv
colcon build --symlink-install --packages-select gate_detection && source install/setup.bash
```

Obtain the v3.4 checkpoint (`best_deeplabv3plus_keypoint_multi_balanced_v3_nextgate_seed42.pth`, trained in the `DroneRacing_GateDetection` repository) and place it in `src/perception/gate_detection/checkpoints/`. With `--symlink-install` it is picked up without rebuilding; otherwise rebuild so `setup.py` installs it to `share/gate_detection/checkpoints/`.

## Run

Standalone:

```bash
ros2 launch gate_detection gate_detection.launch.py \
  checkpoint:=/path/to/best_deeplabv3plus_keypoint_multi_balanced_v3_nextgate_seed42.pth \
  camera_topic:=/cam0/image_rgb camera_info_topic:=/cam0/camera_info max_rate_hz:=30
```

Through the bringup:

```bash
ros2 launch drone_control racing_bringup.launch.py msp_bench_mode:=true view_stream:=false \
  use_gate_detection:=true gate_camera_topic:=/cam0/image_rgb
```

Use the RGB topic. `/cam0/image_raw` is mono8 and carries no colour information for the network.

The AIGP colour detector is started by `aigp_bringup.launch.py`; see [AIGP → Workflow](../aigp_drivers/workflow.md).

## Verify

```bash
ros2 topic hz /gate_detection/drone_pose
ros2 topic echo --once /gate_detection/drone_pose
```

Read the node's log on start: it prints the device, which heads the checkpoint has, whether the parkour map loaded (`parkour map loaded: 6 gates`), and one `[WARN]` per active stub (`cam_T_body STUB in use`, `bootstrap to start_gate_id=0`). Frames without a lock log `no gate locked (reason=..., candidates=N)`.

Offline, without ROS, on a directory of images with the v1 pipeline:

```bash
cd src/perception/gate_detection
python3 gate_detection/scripts/inference_standalone.py checkpoints/best_gatenet.pth --n 12 --out inference.png
```

(The `inference_standalone` and `viewer_standalone` console entry points in `setup.py` reference a `main()` that neither script defines; run them as files.)

The gate-id selector has a pure-numpy desk test:

```bash
python3 src/perception/gate_detection/tests/test_a11_gate_id_selection.py
```

## Troubleshooting

| Symptom | Cause | Fix |
| :--- | :--- | :--- |
| `FileNotFoundError: v3.4 checkpoint not found` | Weights not in the repository | Copy the `.pth` into `checkpoints/` or pass `checkpoint:=` |
| `received image before camera_info; dropping frame` and nothing else, forever | `camera_info` QoS mismatch (node wants transient-local; drivers publish volatile) | Republish `camera_info` with transient-local durability, or change the node's `info_qos` to volatile — the latter is the right fix |
| `has keypoint_head=False` at start | Wrong checkpoint generation | Use a v3.4 multi-head checkpoint |
| `no gate locked` on every frame | Model not confident, or wrong image (mono, upside-down) | Check `gate_camera_topic`, view `/cam0/image_rgb`; lower `HybridTrackerConfig.conf_threshold` only for experiments |
| `unsupported distortion_model` | Camera publishes something other than `equidistant`/`plumb_bob` | Fix the camera's `camera_info` |
| Pose position is tens of metres off | Parkour map is the AIGP course; `start_gate_id` wrong | Provide a lab map via `parkour_map_path`, set `start_gate_id` |
| `_select_world_T_gate: ... got=NN m to gate k` | Prior stale or drone off course | Expected while stubs are active; the node keeps publishing |
| Inference too slow (rate far below `max_rate_hz`) | CPU inference | Check the `device=` line in the log; install CUDA torch on the Jetson |

## Training data tools (v1)

`utils/generate_labels.py`, `utils/augment.py`, `utils/download_backgrounds.py` and `scripts/viewer_standalone.py` build and curate the synthetic dataset used for GateNet. They run as plain Python scripts and are unrelated to the v3.4 model, whose training lives in `DroneRacing_GateDetection`.
