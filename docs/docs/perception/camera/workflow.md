# Camera — workflow

**Owner:** [Ilyas Seckin](https://github.com/ilysec)

## Prerequisites (once per Jetson)

1. Enable the CSI camera in the device tree and reboot:
   ```bash
   sudo /opt/nvidia/jetson-io/jetson-io.py
   # Configure Jetson 24pin CSI connector → Camera IMX219-A
   ```
2. Confirm the sensor streams outside ROS:
   ```bash
   v4l2-ctl --set-fmt-video=width=1280,height=720,pixelformat=RG10 \
     --stream-mmap --stream-count=300 -d /dev/video0
   ```
3. Install the Isaac ROS packages the production path needs (`isaac_ros_argus_camera`, `isaac_ros_image_proc`, optionally `isaac_ros_h264_encoder`):
   ```bash
   python3 scripts/tools/setup_native_dependencies.py
   ```
4. Over Wi-Fi, disable power saving or frame delivery stutters:
   ```bash
   sudo iw dev wlan0 set power_save off
   ```

## Run

Production path, as part of the bringup:

```bash
ros2 launch drone_control racing_bringup.launch.py msp_bench_mode:=true view_stream:=false
```

Production path on its own:

```bash
ros2 launch drone_control nitros_camera.launch.py \
  camera_info_url:=file://$PWD/src/perception/camera_calibration/camera_info.yaml
```

GStreamer node (calibration recordings, or when Isaac ROS is unavailable). `publish_raw` defaults to `false`; pass it explicitly or `/cam0/image_raw` never appears:

```bash
colcon build --symlink-install --packages-select imx219_argus_camera && source install/setup.bash
ros2 launch imx219_argus_camera isaac_camera.launch.py publish_raw:=true color_encoding:=rgb8
```

H.264 as a ROS topic (needs `isaac_ros_h264_encoder`):

```bash
ros2 launch drone_control racing_bringup.launch.py msp_bench_mode:=true \
  view_stream:=true h264_backend:=isaac_ros
```

On x86, without a camera:

```bash
ros2 run imx219_argus_camera test_camera_publisher.py --ros-args -p fps:=30
```

## Verify

```bash
ros2 topic hz /cam0/image_raw          # ~60
ros2 topic hz /cam0/image_rgb          # ~60
ros2 topic echo --once /cam0/camera_info | head -20   # distortion_model, k
ros2 run rqt_image_view rqt_image_view  # pick /cam0/image_rgb, Reliability = Best Effort
```

Check that `camera_info.distortion_model` matches what your consumer expects (`plumb_bob` on the NITROS path, `equidistant` from `imx219_argus_camera`).

## Troubleshooting

| Symptom | Cause | Fix |
| :--- | :--- | :--- |
| Argus times out at start, no frames | Driver waiting for a stereo partner or fsync | Keep `module_id: -1`, `fsync_type: 0` in `nitros_camera.launch.py` |
| `imx219_argus_camera` times out on JetPack 6 | Exposure/gain range set on the ISP | Leave `use_exposure_gain_range: false` |
| "Skipping imx219_argus_camera (Jetson only)" at build | Building on x86 | Expected; use `test_camera_publisher.py` |
| `/cam0/image_raw` missing with `imx219_argus_camera` | `publish_raw` defaults to false | `publish_raw:=true` |
| `rqt_image_view` shows nothing | Reliable subscriber on a best-effort topic | Select Best Effort in the viewer, or `--qos-reliability best_effort` on the CLI |
| `/cam0/image_h264` never appears | `h264_backend:=gstreamer_ros` (default) does nothing on the NITROS path | `h264_backend:=isaac_ros` |
| Image upside-down in OpenVINS but upright in gate detection | Only the RGB branch is flipped | Known; see [Camera → Technical](technical.md#known-limitations) |
| Frame rate collapses when recording `/cam0/image_rgb` | Disk bandwidth | Record `/cam0/image_raw` (mono) or H.264 instead |

## Recording

```bash
ros2 bag record /cam0/image_raw /cam0/camera_info /imu0
```

For calibration bags use `scripts/calibration/record_bag.sh`; note that it currently launches `imx219_argus_camera` without `publish_raw:=true` and aborts with "not publishing 5s after launch" until that argument is added to the script. See [Calibration → Workflow](../calibration/workflow.md).
