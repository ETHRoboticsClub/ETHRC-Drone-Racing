# Camera — technical

**Owner:** [Ilyas Seckin](https://github.com/ilysec)

The racing drones carry one IMX219 wide-angle CSI camera (about 131° horizontal field of view) on a Jetson Orin NX. Two driver implementations exist. Know which one you are running.

| Implementation | Where | Used by | Backend |
| :--- | :--- | :--- | :--- |
| NITROS camera | `src/drone_control/launch/nitros_camera.launch.py` | `racing_bringup.launch.py`, `launch/perception.launch.py` | `isaac_ros_argus_camera` + `isaac_ros_image_proc` (NVIDIA packages, not in this repo) |
| `imx219_argus_camera` | `src/perception/imx219_argus_camera/` | `scripts/calibration/record_bag.sh` | `nvarguscamerasrc` GStreamer pipeline, own C++ node |

Both publish the same topic names under `/cam0`, so downstream nodes do not care which one runs. They differ in encodings, flipping and timestamps, as described below.

## Sensor

IMX219 modes from the Jetson device tree. Mode 4 is the racing mode.

| Mode | Resolution | Max rate |
| :--- | :--- | :--- |
| 0 | 3280×2464 | 21 Hz |
| 1 | 3280×1848 | 28 Hz |
| 2 | 1920×1080 | 30 Hz |
| 3 | 1640×1232 | 30 Hz |
| 4 | 1280×720 | 90 Hz with the patched driver, 60 Hz stock |

The stock driver is limited to 60 Hz; [IMX219 90 Hz tuning](../../drone-setup/imx219_90hz_tuning.md) describes the patch. All launch files default to 60 Hz.

The camera is mounted upside-down on Zephyrus. Each implementation compensates differently (see the flipping notes below).

## NITROS camera (production)

One `component_container_mt` named `nitros_camera_container` with three composable nodes in namespace `cam0`:

| Node | Plugin | Input → output | Notes |
| :--- | :--- | :--- | :--- |
| `argus_mono` | `nvidia::isaac_ros::argus::ArgusMonoNode` | CSI → `image_raw_nitros` (NV12), `camera_info` | `camera_id`, `mode`/`sensor_mode`, `module_id: -1`, `fsync_type: 0`, `camera_info_url` |
| `rgb_flip` | `nvidia::isaac_ros::image_proc::ImageFlipNode` | `image_raw_nitros` → `image_rgb` | `flip_mode: BOTH` = 180° rotation; enabled by `publish_rgb` |
| `format_converter_mono` | `nvidia::isaac_ros::image_proc::ImageFormatConverterNode` | `image_raw_nitros` → `image_raw` (`mono8`) | enabled by `publish_mono`; **not flipped** |

Launch arguments: `sensor_id` (0), `sensor_mode` (4), `image_width` (1280), `image_height` (720), `framerate` (60), `camera_info_url`, `raw_output_encoding` (`nv12`), `publish_mono` (true), `publish_rgb` (true).

`camera_info` is loaded from `camera_info_url`, which defaults to the absolute path of `src/perception/camera_calibration/camera_info.yaml` on the drone (`/home/aero/proj/ETHRC-Drone-Racing/...`). That file holds the plumb_bob fit of the calibration; see [Calibration → Technical](../calibration/technical.md).

`module_id: -1` and `fsync_type: 0` are deliberate: without them the Argus driver waits for a stereo partner and times out (commits `ba8df3e2d`, `4b840ff23`).

## `imx219_argus_camera` (calibration recording)

Executable `argus_camera_node`, launch `isaac_camera.launch.py`, config `config/camera_params.yaml`. Builds a `nvarguscamerasrc ... ! tee` pipeline with one branch per enabled output and pulls frames from `appsink`s.

Published topics (remapped to `/cam0/...` by the launch file):

| Topic | Enabled by | Encoding | QoS |
| :--- | :--- | :--- | :--- |
| `image_raw` | `publish_raw` (**default false**) | `output_encoding` (`mono8`) | sensor data, depth 1 |
| `image_rgb` | `publish_color` (default true) | `color_encoding` — launch default **`rgba8`**, node default `rgb8` | sensor data, depth 1; reliable if `color_reliable` |
| `image_h264` | `publish_h264` (default false) | `CompressedImage`, `format: h264`, byte-stream with SPS/PPS | sensor data, depth 1 |
| `camera_info` | always | from `camera_matrix`, `distortion_coefficients`, `distortion_model` parameters (equidistant fit) | reliable, depth 10 |

`enable_h264_stream` adds an `rtph264pay ! udpsink` branch to `ground_station_ip:stream_port` in addition to (or instead of) the ROS topic.

Parameters worth knowing: `sensor_mode` (4), `framerate` (60), `flip_method` (2 = 180°, applied to **every** branch), `use_exposure_gain_range` (false — enabling it caused capture timeouts on JetPack 6), `exposure_time_range_ns`, `gain_range`, `stream_bitrate` (4 Mbit/s), `iframe_interval` (30).

Timestamps are `now()` taken in the appsink callback, not the capture time. `camera_info` is published once per frame per enabled image branch, so with two branches it appears at twice the frame rate.

The package only builds on aarch64. On x86, `test_camera_publisher.py` publishes a synthetic 1280×720 mono8 pattern on `/cam0/image_raw` for wiring tests.

## H.264 output

`racing_bringup.launch.py` accepts `h264_backend` with three values. With the NITROS camera only one of them produces data:

| `h264_backend` | What runs | Produces |
| :--- | :--- | :--- |
| `isaac_ros` | `isaac_ros_h264_encoder` on `/cam0/image_raw_nitros` | `/cam0/image_h264` |
| `gstreamer_ros` (default) | Nothing — the encoder this option refers to lives inside `imx219_argus_camera`, which the bringup no longer launches | — |
| `gstreamer` | `jetson_h264_publisher` RTP/UDP stream, only when `publish_raw:=false` | UDP to `ground_station_ip:video_port` |

Decoding on the ground station is covered in [H.264 video streaming](../../groundstation/video-streaming.md).

## Files

| Path | Purpose |
| :--- | :--- |
| `src/drone_control/launch/nitros_camera.launch.py` | Production camera launch |
| `src/perception/imx219_argus_camera/src/argus_camera_node.cpp` | GStreamer camera node |
| `src/perception/imx219_argus_camera/launch/isaac_camera.launch.py` | Its launch file (the name predates the NITROS path) |
| `src/perception/imx219_argus_camera/config/camera_params.yaml` | Defaults incl. equidistant intrinsics |
| `src/perception/imx219_argus_camera/config/imx219_720p_60fps_rms066px_kalibr.yaml` | Copy of the calibration in Kalibr format |
| `src/perception/imx219_argus_camera/config/gscam_imx219.yaml` | Unused since gscam2 was removed |
| `src/perception/imx219_argus_camera/scripts/test_camera_publisher.py` | Synthetic publisher for x86 |
| `src/perception/camera_calibration/camera_info.yaml` | plumb_bob intrinsics served by the NITROS path |

## Known limitations

- On the NITROS path `/cam0/image_rgb` is rotated 180° and `/cam0/image_raw` is not. OpenVINS therefore sees upside-down frames while gate detection sees upright ones; the OpenVINS camera–IMU extrinsic does not account for this yet.
- The two implementations publish different `camera_info` (plumb_bob vs equidistant) for the same lens. Both derive from the same calibration; consumers must honour `distortion_model`.
- Neither implementation timestamps frames with a hardware clock shared with the IMU.
- `h264_backend:=gstreamer_ros`, the bringup default, produces no H.264 topic on the NITROS path; the log line claiming otherwise is a leftover.
