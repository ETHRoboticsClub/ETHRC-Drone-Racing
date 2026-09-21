# Calibration — technical

**Owner:** [Mahbod Tajdini](https://github.com/mahbodtajdini)

Every consumer of pixels — OpenVINS, gate detection, the deprecated ArUco node — needs the camera intrinsics, and OpenVINS additionally needs the camera–IMU extrinsic and time offset. This page records what has been calibrated, the numbers, and where they are used. How to redo it is on the [workflow page](workflow.md).

| Calibration | Sensors | Result | Status |
| :--- | :--- | :--- | :--- |
| IMX219 intrinsics | IMX219 at 1280×720, 60 fps, on Zephyrus | `camera_calibration/imx219_720p_60fps_rms066px.*` | Done, April 2026, RMS 0.66 px |
| Camera–IMU extrinsic + time offset | IMX219 + Betaflight IMU | Kalibr `camchain-imucam-*.yaml` | Bags recorded (May 2026, Zephyrus and Boreas), no accepted result in the repository |
| Vicon-to-body offset | Vicon markers vs body frame | | Not done; the Vicon object origin is used as the body origin |

## IMX219 intrinsics

Fit with OpenCV on 80 frames of a 9×6 inner-corner checkerboard (34 mm measured squares), by `scripts/calibration/calibrate_imx219.py`. The full report with method and diagnostics is [imx219_calibration.pdf](assets/imx219_calibration.pdf).

| Parameter | Equidistant (Kannala–Brandt) | Radtan (plumb_bob) |
| :--- | :--- | :--- |
| `fx`, `fy` | 559.382, 559.454 | 567.957, 567.412 |
| `cx`, `cy` | 637.689, 352.823 | 636.255, 347.658 |
| Distortion | `k1..k4` = 0.0670, −0.0285, −0.0182, 0.0587 | `k1,k2,p1,p2,k3` = −0.2746, 0.1055, 0.0020, 0.0012, −0.0218 |
| RMS reprojection | 0.660 px | 0.653 px |
| Field of view | ~131° horizontal, ~150° diagonal | |

Both fits describe the same lens. The equidistant model is the recommended one: a 5-coefficient polynomial in *r* loses accuracy beyond ~120° horizontal field of view, and gates appear at the image edge during turns. The radtan fit exists for tools that only understand `plumb_bob` and for comparison with the TII drone-racing-dataset schema.

![Sample detection](assets/sample_detection.png){ width=420 }
![Reprojection error histogram](assets/reprojection_error_hist.png){ width=420 }

The frames were recorded with the camera mounted upside-down; orientation does not affect the intrinsics.

### Who uses which file

| File | Model | Consumer |
| :--- | :--- | :--- |
| `src/perception/camera_calibration/imx219_720p_60fps_rms066px.json` | both (`mtx`/`dist` radtan, `fisheye` block) | Reference; TII-compatible field names |
| `src/perception/camera_calibration/imx219_720p_60fps_rms066px_kalibr.yaml` | equidistant | Drop-in `cam0` block for OpenVINS `kalibr_imucam_chain.yaml` (not yet dropped in — see [VIO → Technical](../vio/technical.md#configuration)) and seed for Kalibr |
| `src/perception/camera_calibration/camera_info.yaml` | plumb_bob | Served on `/cam0/camera_info` by the NITROS camera (`camera_info_url`) |
| `src/perception/imx219_argus_camera/config/camera_params.yaml` | equidistant | Served on `/cam0/camera_info` by `imx219_argus_camera` |
| `src/perception/imx219_argus_camera/config/imx219_720p_60fps_rms066px_kalibr.yaml` | equidistant | Copy of the Kalibr YAML next to the camera package |

Consequence: the two camera drivers publish different `distortion_model` values. Gate detection handles both; make sure anything new reads `distortion_model` instead of assuming one.

### When to recalibrate

Lens swap or re-mount, sensor swap, a change of sensor mode (resolution or framerate), mechanical impact, large temperature change. Naming convention for results: `<sensor>_<mode>_rms<RMS in 10⁻² px>` so runs are distinguishable.

## Camera-IMU (Kalibr)

Kalibr estimates `T_cam_imu`, the time offset and refined intrinsics from a bag of `/cam0/image_raw` + `/imu0` while the drone is moved in front of an AprilGrid. Targets are in `calibration/targets/` at the repository root (6×6 `t36h11` grids at 22, 30, 45 and 88 mm tag size for A4/A3/A2). Starter IMU noise values for the flight controller's MEMS IMU (used until an Allan-variance analysis exists):

```yaml
accelerometer_noise_density: 2.0e-3   # m/s^2/sqrt(Hz)
accelerometer_random_walk:   3.0e-4   # m/s^3/sqrt(Hz)
gyroscope_noise_density:     1.7e-4   # rad/s/sqrt(Hz)
gyroscope_random_walk:       2.0e-5   # rad/s^2/sqrt(Hz)
update_rate: 150.0                    # must match target_hz of the blackbox driver
```

Acceptance targets for a run: mean reprojection error < 0.7 px, gyro residual < 0.005 rad/s, accel residual < 0.05 m/s², time offset inside the search window (not pegged at the padding), translation within ~1 cm of a ruler measurement.

Bags recorded for this purpose are listed in [ROS bags](../../infrastructure/ros_bags.md) (`imu-cam-calib*`). No result from them has been committed, which is why `zephyrus_real` still carries an identity extrinsic.

## Files

| Path | Purpose |
| :--- | :--- |
| `src/perception/camera_calibration/` | Results, checkerboard PDF, diagnostic images |
| `scripts/calibration/generate_checkerboard.py` | Renders the 9×6 (inner corners) 35 mm A3 board |
| `scripts/calibration/record_bag.sh` | Records `/cam0/image_raw` + `/cam0/camera_info` on the drone |
| `scripts/calibration/calibrate_imx219.py` | Bag → both fits → JSON, Kalibr YAML, histogram |
| `calibration/targets/` | Kalibr AprilGrid PDFs and YAMLs |
| `docs/imx219_calibration/` | LaTeX source of the report |
