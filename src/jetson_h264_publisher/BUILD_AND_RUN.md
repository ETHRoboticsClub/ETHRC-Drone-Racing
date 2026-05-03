# Jetson H.264 Publisher — build and run

Tested target: **Jetson Orin NX 16GB**, JetPack 6.x, Ubuntu 22.04, ROS 2 Humble.

## 1. System packages (one-time)

```bash
sudo apt update
sudo apt install -y \
  python3-gi gir1.2-gstreamer-1.0 \
  gstreamer1.0-tools gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good gstreamer1.0-plugins-bad
```

The Jetson NVENC plugins (`nvv4l2h264enc`, `nvv4l2h265enc`, `nvarguscamerasrc`)
ship with JetPack — do **not** apt-install a generic `gstreamer1.0-plugins-bad`
build that would shadow them. Verify:

```bash
gst-inspect-1.0 nvv4l2h264enc | head
gst-inspect-1.0 nvarguscamerasrc | head
```

## 2. Get `h264_image_transport` / `h264_msgs`

```bash
cd ~/ros2_ws/src
git clone https://github.com/clydemcqueen/h264_image_transport.git
```

It builds both `h264_msgs` and the transport plugin.

## 3. Drop this package in and build

```bash
cp -r jetson_h264_publisher ~/ros2_ws/src/
cd ~/ros2_ws
colcon build --symlink-install --packages-select h264_msgs h264_image_transport jetson_h264_publisher
source install/setup.bash
```

## 4. Tune the Jetson for streaming

```bash
sudo nvpmodel -m 0          # MAXN
sudo jetson_clocks
```

## 5. Run

```bash
ros2 launch jetson_h264_publisher encoder.launch.py
```

You should see `image_raw/h264` publishing at your configured framerate:

```bash
ros2 topic hz /image_raw/h264
ros2 topic bw /image_raw/h264
```

## 6. View on a ground station

On the GCS (any x86 / ARM box), republish to raw using
`h264_image_transport`'s republish node. 

> [!IMPORTANT]
> The Jetson publisher uses **Best Effort** QoS. You **must** override the subscriber's QoS settings, otherwise ROS 2 will not connect them (you will see an `incompatible QoS` warning).

```bash
ros2 run image_transport republish h264 \
  --ros-args \
  --remap in/h264:=/image_raw/h264 \
  --remap out:=/image_raw \
  -p qos_overrides./image_raw/h264.subscription.reliability:=best_effort
```

Now you can view the raw stream:
```bash
ros2 run rqt_image_view rqt_image_view /image_raw
```

## 7. Viewing in Foxglove Studio (Recommended)

Foxglove Studio is the easiest way to view the H.264 stream without needing a local republisher.

1.  Open **Foxglove Studio** and connect to your ROS 2 stack.
2.  Add a **"Raw Video"** panel to your layout.
3.  Select the `/image_raw/h264` topic.
4.  In the panel settings (gear icon), set the **Format** to `h264`.
5.  Foxglove handles the QoS automatically.

---

## Tuning notes

| Knob | Default | Effect |
|---|---|---|
| `bitrate` | 4 Mbps | Match to ~70% of usable radio-link bandwidth. |
| `peak_bitrate` | 6 Mbps | Hard cap for VBR; protect the radio link from IDR spikes. |
| `iframe_interval` | 15 | Lower = faster recovery for late subscribers, more bandwidth. |
| `preset_level` | 1 (UltraFast) | Trade quality for latency. 1 is right for teleop. |
| `control_rate` | 1 (VBR) | Use 2 (CBR) if your link is bandwidth-constrained and stable. |

## DDS / radio link tips

- Use BEST_EFFORT QoS on the subscriber side too (the publisher already does).
- If you see fragmented or dropped frames over Wi-Fi, raise the
  `RMW` socket buffers and DDS `max_message_size`. For Cyclone DDS you
  can set `<MaxMessageSize>65500B</MaxMessageSize>` in your config.
- If frames at 1080p exceed UDP MTU after fragmentation, consider
  sliced encoding (`slice-header-spacing`) or step down to 720p.

## Switching to H.265

H.265 will give you ~30% better quality at the same bitrate but the
`h264_image_transport` package only handles H.264. Either:

1. Use the equivalent `h265_image_transport` (community fork), or
2. Encode H.265 and publish as `sensor_msgs/CompressedImage` with
   `format: "h265"`. Most viewers won't decode it directly, but you can
   stream-decode on your GCS with another GStreamer node.

In `encoder_node.py`, swap `nvv4l2h264enc` for `nvv4l2h265enc` and
`h264parse` for `h265parse`.
