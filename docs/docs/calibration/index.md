# Calibration Overview

Sensor calibration is a prerequisite for every component downstream of the camera and IMU — visual-inertial odometry, gate detection, ArUco pose estimation, and the EKF fusion layer all assume that the camera intrinsics and (eventually) the camera↔IMU extrinsics are known and accurate.

This section collects the calibration procedures and results used by the racing drones.

## What's covered

| Calibration | Sensors | Output | Status |
| --- | --- | --- | --- |
| [IMX219 wide-angle intrinsics](imx219_intrinsics.md) | IMX219 CSI camera @ 1280×720 60 fps | TII-style JSON + Kalibr YAML | **Done** — RMS 0.66 px |
| Camera↔IMU joint calibration | IMX219 + Betaflight IMU | Kalibr `imucam_chain.yaml` (`T_cam_imu` + time offset) | Not started |
| Stereo extrinsics | (if a second IMX219 is added) | Kalibr stereo YAML | Not planned |

## Why this matters

- **Intrinsics** (`fx, fy, cx, cy` + distortion) define how a 3-D ray maps to a pixel. Wrong intrinsics translate to systematic errors in every measurement that uses pixel positions: gate-corner reprojection, marker pose estimation, VIO triangulation.
- **Camera↔IMU extrinsics** (`T_cam_imu` + time offset) are needed by visual-inertial odometry to fuse the two streams correctly. Until these are calibrated, OpenVINS will either refuse to converge or produce drifting state estimates.
- **Re-calibration triggers**: lens swap, lens re-mount, sensor swap, sensor mode (resolution / framerate) change, large temperature shifts, mechanical impact. When in doubt, re-run.

## File-naming convention

Calibration outputs live under [`camera_calibration/`](https://github.com/ETHRoboticsClub/ETHRC-Drone-Racing/tree/main/camera_calibration) at the repo root and are mirrored into the relevant ROS package config directories. File names embed the sensor, the resolution / mode, and the achieved fisheye reprojection RMS in 10⁻² px (e.g. `rms066px` ⇒ 0.66 px), so multiple runs are immediately distinguishable.

```
imx219_720p_60fps_rms066px.json          ← TII-compatible schema
imx219_720p_60fps_rms066px_kalibr.yaml   ← OpenVINS-compatible schema
```

The Kalibr-style YAML drops straight into the existing `src/perception/VIO/open_vins/config/<dataset>/kalibr_imucam_chain.yaml` slot.

## Tooling

The calibration scripts under [`scripts/calibration/`](https://github.com/ETHRoboticsClub/ETHRC-Drone-Racing/tree/main/scripts/calibration) are designed to be run with [`uv`](https://docs.astral.sh/uv/) so that no system-Python pollution is needed. Each script declares its own dependencies inline (PEP 723) and `uv` resolves them into an ephemeral environment.

```bash
# One-time install of uv (no sudo required)
curl -LsSf https://astral.sh/uv/install.sh | sh
export PATH="$HOME/.local/bin:$PATH"
```

For the recording side on the drone, use the current Argus camera package rather
than the deprecated `gscam2` path. Build and source the camera package before
recording:

```bash
colcon build --symlink-install --packages-select imx219_argus_camera
source install/setup.bash
```
