# Kalibr IMU–Camera Calibration

**Page Owner:** [Giacomo Piatelli](https://github.com/CornelioP)

---

## Overview

This tutorial walks through producing an IMU-to-camera extrinsic calibration
(`T_cam0_imu0`) and time offset suitable for [OpenVINS](vio.md), using
[Kalibr](https://github.com/ethz-asl/kalibr).

The guide assumes you **already have a rosbag** containing synchronized
`/cam0/image_raw` and `/imu0` topics recorded while moving the drone in front
of a printed AprilGrid target. For bag recording and the rest of the VIO
pipeline wiring, see [VIO Synchronization](../drone-setup/vio_synchronization.md).

> [!NOTE]
> The calibration itself is run **off-board** on a laptop/workstation. The
> Jetson only produces the rosbag. Kalibr is RAM-hungry and ROS 1 native, so
> running it in Docker on a desktop is the supported path.

---

## Inputs

Before you start, you should have:

| Artifact | Where it comes from |
| :--- | :--- |
| `calib.bag` (or a ROS 2 bag dir) | Recorded on the drone with `/cam0/image_raw` + `/imu0` |
| AprilGrid printout | e.g. `KalibrGrid A3: 6x6 t36h11, tagSize=0.030 m, tagSpacing=0.3` |
| Camera intrinsics YAML | `camera_calibration/imx219_720p_60fps_rms066px_kalibr.yaml` in this repo |
| Laptop with Docker installed | Kalibr ships as an OCI image |

---

## 1. Prepare the working directory

Put the bag and the three Kalibr config YAMLs in a single folder so Docker
can mount it as `/data`.

```bash
mkdir -p ~/kalibr_run && cd ~/kalibr_run
# copy bag here (rsync from the drone, see Section 3 below)
```

### 1.1 `target.yaml` — AprilGrid description

Parameters must match the **printed** target exactly. Example for the A3
Kalibr grid (`6x6 t36h11, tagsize=0.030 m, tagspacing=0.3`):

```yaml
target_type: 'aprilgrid'
tagCols: 6
tagRows: 6
tagSize: 0.030
tagSpacing: 0.3
tagFamily: 't36h11'
```

> [!IMPORTANT]
> `tagSize` is in **meters** and refers to the black square edge, not the
> full tile including the white border. `tagSpacing` is the ratio
> `(gap between tags) / (tag size)`. Double-check with a ruler on the
> printout before running.

### 1.2 `cam_chain.yaml` — Camera intrinsics seed

Copy from the existing repo calibration (Kannala–Brandt / equidistant, valid
at any framerate for the IMX219 @ 1280×720):

```yaml
cam0:
  camera_model: pinhole
  intrinsics: [559.3817837, 559.4544914, 637.6885196, 352.8232945]
  distortion_model: equidistant
  distortion_coeffs: [0.0670286312, -0.02854181394, -0.01821324581, 0.05872486157]
  resolution: [1280, 720]
  rostopic: /cam0/image_raw
```

These intrinsics will be refined jointly with the extrinsics during
`kalibr_calibrate_imu_camera`.

### 1.3 `imu.yaml` — IMU noise model

Starter values for a consumer-grade MEMS IMU (ICM-42688 / BMI270 class) as
found on typical Betaflight FCs:

```yaml
rostopic: /imu0
update_rate: 100.0           # Hz — must match the actual publish rate

accelerometer_noise_density: 2.0e-3       # [m/s^2 / sqrt(Hz)]
accelerometer_random_walk:   3.0e-4       # [m/s^3 / sqrt(Hz)]
gyroscope_noise_density:     1.7e-4       # [rad/s / sqrt(Hz)]
gyroscope_random_walk:       2.0e-5       # [rad/s^2 / sqrt(Hz)]
```

> [!TIP]
> For a production-grade calibration, replace these with values obtained
> from an **Allan-variance analysis** on a stationary 2h log. For first
> bring-up the defaults above are fine — Kalibr is robust to moderate
> miscalibration of noise.

---

## 2. Convert ROS 2 bag → ROS 1 (if needed)

Kalibr expects a ROS 1 `.bag` file. If you recorded with `ros2 bag record`,
convert first:

```bash
pip install --user rosbags
export PATH="$HOME/.local/bin:$PATH"

rosbags-convert calib_bag_dir --dst calib.bag
```

> [!NOTE]
> The CLI recently changed — if you see `unrecognized arguments: --src`,
> pass the source path positionally as shown above.

---

## 3. Transfer the bag from the drone

The fastest path is the Jetson USB device-mode link (no Wi-Fi involved).
Plug the micro-USB cable from the Jetson into the laptop. A virtual ethernet
gadget appears — the Jetson is at `192.168.55.1`, the laptop at
`192.168.55.100`.

```bash
# from the laptop
rsync -avP --compress-level=0 aero@192.168.55.1:~/calib_bag ./
```

Expect ~40–50 MB/s sustained. A 90-second RGB8 720p bag is ≈ 5.9 GB.

> [!TIP]
> To cut the bag size by ~3×, change the camera pipeline to output `GRAY8`
> instead of `RGB` in `imx219_argus_camera`'s GStreamer chain. Kalibr
> converts to grayscale anyway.

---

## 4. Run Kalibr

Pull the image once:

```bash
docker pull stereolabs/kalibr:latest
```

Run the IMU–camera calibration. The `-w /data` flag is important — without
it, Kalibr writes its outputs inside the container and they disappear on
exit:

```bash
docker run -it --rm \
  -v "$PWD":/data \
  -w /data \
  -e MPLBACKEND=Agg \
  stereolabs/kalibr:latest \
  kalibr_calibrate_imu_camera \
    --bag calib.bag \
    --cam cam_chain.yaml \
    --imu imu.yaml \
    --target target.yaml \
    --timeoffset-padding 0.1 \
    --dont-show-report
```

| Flag | Why |
| :--- | :--- |
| `-w /data` | Kalibr writes output files to CWD. Point CWD at the mount. |
| `-e MPLBACKEND=Agg` | Forces matplotlib to render to file instead of a window. Without it the PDF report step crashes with `no display name and no $DISPLAY`. |
| `--dont-show-report` | Skip the interactive viewer (you have no X in the container). |
| `--timeoffset-padding 0.1` | Search window for the camera–IMU time offset (±100 ms). USB-VCP polling easily needs this. |

Expect 10–20 minutes of runtime on a modern laptop, dominated by AprilGrid
corner extraction on every frame.

---

## 5. Interpret the output

Kalibr writes four files into `~/kalibr_run/`:

| File | Contents |
| :--- | :--- |
| `camchain-imucam-*.yaml` | Drop-in for OpenVINS (full camera chain + `T_cam_imu`). |
| `imu-*.yaml` | IMU noise parameters used. |
| `results-imucam-*.txt` | Human-readable summary (residuals, extrinsic, time offset). |
| `report-imucam-*.pdf` | Per-axis residual plots. |

Open `results-imucam-*.txt` and sanity-check against these targets:

| Metric | Good | Excellent |
| :--- | :--- | :--- |
| Mean reprojection error | < 0.7 px | < 0.3 px |
| Gyroscope residual | < 0.005 rad/s | < 0.001 rad/s |
| Accelerometer residual | < 0.05 m/s² | < 0.02 m/s² |
| Time offset (`t_imu − t_cam`) | within ±50 ms, not pegged at padding | — |
| `T_cam_imu` translation | matches ruler measurement within ~1 cm | — |

If the time offset is pegged at exactly ±100 ms, rerun with
`--timeoffset-padding 0.2`. If residuals are normalized far above 1.0,
your IMU noise values are too strict — loosen them by 5–10× and rerun.

---

## 6. Integrate into OpenVINS

Copy the generated camera chain into the OpenVINS config for your drone:

```bash
cp camchain-imucam-*.yaml \
   <repo>/src/perception/VIO/open_vins/config/<your_drone>/camchain.yaml
```

Then point the OpenVINS `estimator_config.yaml` at it — see [VIO](vio.md)
for how `config_path` is wired.

---

## Common issues

| Symptom | Cause | Fix |
| :--- | :--- | :--- |
| `no display name and no $DISPLAY` | Docker has no X, matplotlib tries to open a window for the PDF | Add `-e MPLBACKEND=Agg` and `--dont-show-report` |
| No output files on host | Kalibr wrote to the container's CWD | Add `-w /data` |
| "Not enough corners detected" | Target not fully visible, motion blur, bad lighting | Re-record; slower motion, better lighting |
| Optimizer diverges | IMU noise values far off, or bag starts/ends mid-motion | Loosen noise params, trim bag, ensure 2–3 s still at start |
| Time offset hits padding limit | Offset is larger than search window | Raise `--timeoffset-padding` to 0.2 or higher |
| Rosbag conversion error `unrecognized arguments: --src` | Newer `rosbags` CLI | Pass source positionally: `rosbags-convert <dir> --dst x.bag` |

---

## Next Steps

- Validate the calibration by running OpenVINS live with the generated
  `camchain-imucam-*.yaml` and inspecting the online-calibration drift.
- Replace the starter IMU noise model with Allan-variance-derived values
  (`kalibr_allan` on a 2h stationary log).
- Once the [Blackbox IMU UART](../drone-setup/vio_synchronization.md) is
  wired, re-run the calibration at 2 kHz IMU with HTE timestamps for a
  production-grade result.
