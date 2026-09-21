# Calibration — workflow

**Owner:** [Mahbod Tajdini](https://github.com/mahbodtajdini)

## Intrinsics (IMX219)

The scripts run with [`uv`](https://docs.astral.sh/uv/) and declare their own dependencies (PEP 723), so no system Python packages are needed:

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh && export PATH="$HOME/.local/bin:$PATH"
```

### 1. Target

The A3 checkerboard (9×6 inner corners, 35 mm nominal) is `src/perception/camera_calibration/checkerboard_9x6_35mm_a3.pdf`; the printed and mounted copy is kept by the team (ask Dimetri). To re-print:

```bash
./scripts/calibration/generate_checkerboard.py --square-mm 35
```

**Measure one square after printing.** Printers scale by 1–3 %; the existing board measures 34.0 mm and that value is what the fit needs.

### 2. Record on the drone

`record_bag.sh` launches `imx219_argus_camera` and records `/cam0/image_raw` + `/cam0/camera_info` into `bags/imx219_calib_<timestamp>` (or a name you pass):

```bash
ssh aero@zephyrus
cd ~/proj/ETHRC-Drone-Racing && source install/setup.bash
./scripts/calibration/record_bag.sh
```

The script does not pass `publish_raw:=true`, which the camera launch now needs; until it is updated, launch the camera yourself and record manually:

```bash
ros2 launch imx219_argus_camera isaac_camera.launch.py publish_raw:=true sensor_mode:=4 framerate:=60 &
ros2 bag record -o bags/imx219_calib_$(date +%Y%m%d_%H%M%S) /cam0/image_raw /cam0/camera_info
```

Record 60–120 s: board flat, then tilted ±30° about each axis, then pushed into each image corner, moving slowly (about one board width per second). A live preview helps:

```bash
ros2 run web_video_server web_video_server --ros-args -p port:=8080
# laptop: http://<drone-ip>:8080/stream?topic=/cam0/image_raw&type=mjpeg
```

### 3. Fit

On the drone (avoids copying a multi-GB bag):

```bash
./scripts/calibration/calibrate_imx219.py --bag bags/imx219_calib_<timestamp> --square-size-mm 34.0 \
  --out-dir src/perception/camera_calibration
```

Options: `--frame-stride 10`, `--max-frames 80`, `--out-name imx219_720p_60fps` (`--out-dir` defaults to `camera_calibration/` at the repository root, which no longer exists in the mono-repo). The script writes `<name>_rms<RMS>px.json`, `<name>_rms<RMS>px_kalibr.yaml` and `reprojection_error_hist.png` into `--out-dir`.

### 4. Publish the result

```bash
scp 'aero@zephyrus:proj/ETHRC-Drone-Racing/src/perception/camera_calibration/imx219_720p_60fps*' src/perception/camera_calibration/
```

Then update every consumer listed in [Calibration → Technical → Who uses which file](technical.md#who-uses-which-file): `camera_info.yaml` (radtan values), `imx219_argus_camera/config/camera_params.yaml` (equidistant values), the copy of the Kalibr YAML, and the OpenVINS `kalibr_imucam_chain.yaml`.

## Camera-IMU calibration (Kalibr)

Run off-board in Docker; Kalibr is ROS 1 and memory-hungry.

### 1. Record

On the drone, with the camera and IMU driver running (see [Camera → Workflow](../camera/workflow.md), [IMU → Workflow](../imu/workflow.md)):

```bash
ros2 bag record -o bags/imu_cam_calib_$(date +%Y%m%d) /cam0/image_raw /imu0
```

Start with 2–3 s at rest, then excite all six axes in front of the AprilGrid for 60–90 s, keeping the grid fully in view. Use a grid from `calibration/targets/` and note which one.

### 2. Transfer and convert

Over the Jetson USB device link (Jetson `192.168.55.1`, laptop `192.168.55.100`; ~40–50 MB/s):

```bash
rsync -avP --compress-level=0 aero@192.168.55.1:~/proj/ETHRC-Drone-Racing/bags/imu_cam_calib_<date> ./
pip install --user rosbags && rosbags-convert imu_cam_calib_<date> --dst calib.bag
```

### 3. Configuration files

In one directory with `calib.bag`:

`target.yaml` — match the printed grid (values for `aprilgrid_6x6_30mm_30_a3`):

```yaml
target_type: 'aprilgrid'
tagCols: 6
tagRows: 6
tagSize: 0.030        # metres, black square edge
tagSpacing: 0.3       # gap / tagSize
tagFamily: 't36h11'
```

`cam_chain.yaml` — copy `src/perception/camera_calibration/imx219_720p_60fps_rms066px_kalibr.yaml`.

`imu.yaml` — the starter noise model from [Calibration → Technical](technical.md#camera-imu-kalibr) with `update_rate` equal to the driver's `target_hz`.

### 4. Run

```bash
docker pull stereolabs/kalibr:latest
docker run -it --rm -v "$PWD":/data -w /data -e MPLBACKEND=Agg stereolabs/kalibr:latest \
  kalibr_calibrate_imu_camera --bag calib.bag --cam cam_chain.yaml --imu imu.yaml \
    --target target.yaml --timeoffset-padding 0.1 --dont-show-report
```

`-w /data` keeps the outputs on the host; `MPLBACKEND=Agg` and `--dont-show-report` avoid the headless-matplotlib crash. Expect 10–20 min.

### 5. Read the result

`results-imucam-*.txt` has the residuals, `T_cam_imu` and the time offset; compare against the acceptance targets on the technical page. `camchain-imucam-*.yaml` is the OpenVINS input: copy its `cam0` block (including `T_cam_imu`) into `src/perception/VIO/open_vins/config/zephyrus_real/kalibr_imucam_chain.yaml` and the time offset into `kalibr_imu_chain.yaml`, then commit inside the submodule.

## Troubleshooting

| Symptom | Cause | Fix |
| :--- | :--- | :--- |
| `record_bag.sh` aborts: `/cam0/image_raw not publishing` | `publish_raw` not passed | Launch manually as above, or add the argument to the script |
| `calibrate_imx219.py` finds few corners | Motion blur, board partly outside the image | Slower motion; drop `--frame-stride` |
| RMS much higher than 0.7 px | Wrong square size or bent board | Re-measure the print |
| Kalibr: `no display name and no $DISPLAY` | Headless matplotlib | `-e MPLBACKEND=Agg --dont-show-report` |
| Kalibr: no output files on the host | Wrote into the container | `-w /data` |
| Kalibr: time offset pegged at ±padding | Offset larger than the window | `--timeoffset-padding 0.2` |
| Kalibr: normalized residuals ≫ 1 | IMU noise values too tight | Loosen by 5–10× |
| `rosbags-convert: unrecognized arguments: --src` | Newer CLI | Pass the source positionally |
