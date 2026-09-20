# AIGP Camera Driver

**Page Owner:** [Jules](https://github.com/juliomathis)

## Overview

Adapter node for the **AI Grand Prix Virtual Qualifier** simulator vision stream.
```
AIGP simulator -> UDP:5600 chunked JPEG -> vision_driver_node ──▶ /cam0/image_raw   (mono8)
                                                              ├─▶ /cam0/image_rgb   (rgb8)
                                                              └─▶ /cam0/camera_info (pinhole)
```

A C++ `rclcpp` component (composable, intra-process zero-copy). Decoding and
publishing are **lazy** — skipped entirely when a stream has no subscribers — and
each image is built in place (colour conversion written straight into the message
buffer, no `cv_bridge` copy) and moved out as `unique_ptr`.

## Wire format (spec sec. 4.6)

Each UDP packet = a 24-byte little-endian metadata header + a JPEG slice:

| Field          | Type     | Bytes | Description                                  |
|----------------|----------|-------|----------------------------------------------|
| `frame_id`     | `uint32` | 4     | Unique sequence ID for the image frame       |
| `chunk_id`     | `uint16` | 2     | Index of this packet within the frame        |
| `total_chunks` | `uint16` | 2     | Number of packets that make up the frame     |
| `jpeg_size`    | `uint32` | 4     | Size of the reconstructed JPEG (bytes)       |
| `payload_size` | `uint32` | 4     | Size of the JPEG slice in this packet        |
| `sim_time_ns`  | `uint64` | 8     | Simulation timestamp (nanoseconds)           |


## Camera model (spec sec. 3.8)

Pinhole, **no distortion**, 640×360, `[cx, cy] = [320, 180]`, `[fx, fy] = [320, 320]`,
VFoV 90°. The camera shares the body origin but is tilted **20° upward** — that
extrinsic lives in the OpenVINS config (VIO owner), not here.

## Usage

```bash
ros2 launch aigp_camera_driver aigp_camera.launch.py
```

## Verifying & viewing

The image topics use **best-effort** (sensor) QoS — a default *reliable*
subscriber receives nothing, so always select best-effort.

```bash
ros2 topic hz /cam0/image_raw                                   # rate (tolerates best-effort)
ros2 topic echo --once --qos-reliability best_effort /cam0/camera_info
```

```bash
sudo apt install -y ros-humble-rqt-image-view
ros2 run rqt_image_view rqt_image_view   # refresh, pick /cam0/image_rgb, Reliability = Best Effort
```


## Network tuning for the color stream
Raise the host's socket buffers. With native Docker Engine + `network_mode:
host`, the container shares the **WSL host** network namespace, so set this on the
WSL host:

```bash
sudo sysctl -w net.core.rmem_max=2147483647
sudo sysctl -w net.core.wmem_max=2147483647
# persist across reboots:
echo -e "net.core.rmem_max=2147483647\nnet.core.wmem_max=2147483647" | \
  sudo tee /etc/sysctl.d/60-cyclonedds.conf
sudo sysctl --system
```