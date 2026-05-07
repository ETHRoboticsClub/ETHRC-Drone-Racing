# Jetson H.264 RTP Streamer

Target: Jetson Orin NX, JetPack 6.x, Ubuntu 22.04, ROS 2 Humble.

This package is now the GStreamer RTP fallback. The default racing bringup uses
`isaac_ros_h264_encoder` (`h264_backend:=isaac_ros`) so H.264 compression stays
inside the ROS/NITROS graph.

This package now streams the CSI camera directly through the Jetson ISP and
NVENC:

```text
nvarguscamerasrc -> NVMM NV12 1280x720@60 -> nvv4l2h264enc -> RTP/H.264 UDP
```

It does not subscribe to `/cam0/image_raw`. That ROS raw-image hop copies every
frame through CPU memory and is the reason the old implementation could not hold
60 FPS.

## One-Time Jetson Setup

Enable the IMX219 in Jetson IO on JetPack 6.2, then reboot:

```bash
sudo /opt/nvidia/jetson-io/jetson-io.py
```

Select `Configure Jetson 24pin CSI connector`, then `Camera IMX219-A`.

Verify the sensor mode:

```bash
v4l2-ctl --set-fmt-video=width=1280,height=720,pixelformat=RG10 \
  --stream-mmap --stream-count=300 -d /dev/video0
```

Disable Wi-Fi power save before flight:

```bash
sudo iw dev wlan0 set power_save off
```

To persist it, add a root cron entry:

```bash
sudo crontab -e
```

```cron
@reboot /usr/sbin/iw dev wlan0 set power_save off
```

## Build

```bash
colcon build --symlink-install --packages-select jetson_h264_publisher
source install/setup.bash
```

## Run On The Jetson

Replace the host with the ground-station IPv4 address:

```bash
ros2 launch jetson_h264_publisher encoder.launch.py \
  ground_station_ip:=10.200.117.1 \
  port:=5000
```

Default stream settings are `1280x720 @ 60 FPS`, 4 Mbps, SPS/PPS insertion,
and `sync=false async=false`.

## Run The Ground-Station Receiver

Use GStreamer directly so no player-side jitter buffer adds latency:

```bash
gst-launch-1.0 \
  udpsrc port=5000 caps='application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96' \
  ! rtpjitterbuffer latency=0 drop-on-latency=true \
  ! rtph264depay \
  ! avdec_h264 \
  ! videoconvert \
  ! autovideosink sync=false async=false
```

## Bringup Integration

The racing bringup prioritizes perception by default: the CSI camera publishes
`/cam0/image_raw` for OpenVINS and gate detection, and this direct RTP streamer
stays off.

Use the RTP streamer only when you explicitly want the GStreamer fallback:

```bash
ros2 launch drone_control racing_bringup.launch.py \
  view_stream:=true \
  h264_backend:=gstreamer \
  ground_station_ip:=10.200.117.1
```

The perception path is:

```bash
ros2 launch drone_control racing_bringup.launch.py \
  publish_raw:=true
```

## Useful Tuning

| Parameter | Default | Effect |
|---|---:|---|
| `bitrate` | `4000000` | Target bits per second. |
| `iframe_interval` | `30` | IDR interval; lower recovers faster after packet loss. |
| `preset_level` | `0` | Lowest-latency NVENC preset from the research notes. |
| `control_rate` | `1` | Encoder control-rate setting. |
| `encoder_options` | `tune=zerolatency` | Extra `nvv4l2h264enc` options appended to the encoder. |

If your JetPack build rejects `tune=zerolatency` for `nvv4l2h264enc`, launch
with `encoder_options:=` and keep the other low-latency parameters.
