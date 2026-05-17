# H.264 Video Streaming

**Page Owner:** [Julian Hoffman](https://github.com/JulesHere)

The default racing video path uses the Jetson hardware encoder inside
`imx219_argus_camera` and publishes H.264 as a ROS topic. This page focuses on
viewing and recording the stream; the camera-side architecture is documented in
[Jetson Camera Pipeline](../perception/camera_pipeline.md).

The live ROS graph is:

```text
IMX219 CSI
  -> /cam0/image_raw   mono8  -> VIO
  -> /cam0/image_rgb   rgb8   -> gate detection / debug
  -> nvv4l2h264enc            -> /cam0/image_h264
```

`/cam0/image_raw` stays `mono8` because OpenVINS requests `MONO8`. The RGB
branch is the real color camera topic and is best-effort so slow debug tools do
not back-pressure the camera. The default H.264 path does not consume raw RGB.

`/cam0/image_h264` is not a viewable image topic. It is H.264 data in
`sensor_msgs/CompressedImage`. Decode it to `/image_decoded` before opening it in
an image viewer.

## Isaac ROS Packages

The default `h264_backend:=gstreamer_ros` path does not need the Isaac ROS
encoder on the Jetson. Install the decoder on the ground station inside the same
ROS/Isaac environment used to run the stack:

```bash
sudo apt-get update
sudo apt-get install -y ros-humble-isaac-ros-h264-decoder
```

If the decoder is built from source instead, source that Isaac ROS workspace
before this project workspace. The older `h264_backend:=isaac_ros` encoder path
is still available for compatibility.

## Jetson Setup

Complete the hardware and camera checks in
[Jetson Camera Pipeline](../perception/camera_pipeline.md#jetson-setup) first.

## Drone Command

```bash
ros2 launch drone_control racing_bringup.launch.py \
  msp_bench_mode:=true \
  view_stream:=true \
  h264_backend:=gstreamer_ros \
  use_gate_detection:=false
```

Useful checks on the Jetson:

```bash
ros2 topic hz /cam0/image_raw
ros2 topic hz /cam0/image_rgb
ros2 topic hz /cam0/image_h264
```

For recording, `/cam0/image_h264` is the high-rate color video topic. Avoid
bagging `/cam0/image_rgb` except for short debugging runs.

Do not view `/cam0/image_h264` directly in `rqt_image_view`, Foxglove, or RViz
as a normal image. It is H.264 bytes in `sensor_msgs/CompressedImage`, so generic
image viewers may show a black/error placeholder. Decode it first:

```bash
ros2 launch drone_control racing_bringup.launch.py \
  msp_bench_mode:=true \
  view_stream:=true \
  h264_backend:=gstreamer_ros \
  decode_h264:=true \
  use_gate_detection:=false \
  use_vio:=false
```

Then view `/image_decoded`.

For flight logs, record the compressed topic rather than raw RGB:

```bash
ros2 bag record /cam0/image_h264 /cam0/camera_info
```

## Ground-Station Command

Run the same bringup on a non-Jetson host to decode the ROS/DDS H.264 topic:

```bash
ros2 launch drone_control racing_bringup.launch.py \
  view_stream:=true \
  h264_backend:=gstreamer_ros
```

The ground station remaps:

```text
/cam0/image_h264 -> isaac_ros_h264_decoder -> /image_decoded
```

## GStreamer Fallback

For low-level UDP/RTP debugging without Isaac ROS Compression:

```bash
ros2 launch drone_control racing_bringup.launch.py \
  msp_bench_mode:=true \
  view_stream:=true \
  h264_backend:=gstreamer \
  use_gate_detection:=false
```

On the ground station:

```bash
gst-launch-1.0 \
  udpsrc port=5000 caps='application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96' \
  ! rtpjitterbuffer latency=0 drop-on-latency=true \
  ! rtph264depay \
  ! avdec_h264 \
  ! videoconvert \
  ! autovideosink sync=false async=false
```

## Parameters

| Parameter | Default | Description |
|---|---:|---|
| `h264_backend` | `gstreamer_ros` | `gstreamer_ros` for in-camera-node NVENC H.264 ROS topic, `isaac_ros` for the older Isaac encoder path, `gstreamer` for RTP/UDP fallback. |
| `isaac_h264_qp` | `18` | Isaac ROS H.264 quality knob. Only used when `h264_backend:=isaac_ros`. |
| `isaac_h264_iframe_interval` | `30` | I-frame cadence in frames. At 60 Hz, `30` inserts a keyframe roughly every 0.5 s. |
| `isaac_h264_profile` | `1` | H.264 profile passed to Isaac ROS encoder. Keep the default unless decoder compatibility requires a change. |
| `isaac_h264_hw_preset` | `0` | Hardware preset passed to Isaac ROS encoder. Keep the default for low-latency Jetson operation. |
| `video_bitrate` | `4000000` | Target bitrate for `gstreamer_ros` and `gstreamer` backends. |
| `video_width` | `1280` | Encoded width. |
| `video_height` | `720` | Encoded height. |
| `video_fps` | `60` | Camera frame rate. |
| `publish_raw` | `true` | Publish `/cam0/image_raw` for VIO. |
| `publish_rgb` | `true` | Publish `/cam0/image_rgb` as the default color camera topic. |
| `gate_camera_topic` | `/cam0/image_rgb` | Gate detector input. Use RGB for real color; `/cam0/image_raw` is mono. |
