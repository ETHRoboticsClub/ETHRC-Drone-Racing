# H.264 Video Streaming

**Responsible:** Aaron

To achieve low-latency teleoperation and monitoring, the ETHRC Drone Racing project uses a hardware-accelerated H.264 video pipeline. This avoids the high CPU usage of raw image transport over the radio link.

## Pipeline Overview

1.  **Publisher (Jetson)**: The `jetson_h264_publisher` node captures CSI camera frames using the Jetson's hardware encoder and publishes `h264_msgs/msg/Packet`.
2.  **Transport**: The stream is sent over the network using **Best Effort** QoS to minimize latency.
3.  **Republisher (Ground Station)**: The `image_transport/republish` node (with the `h264` plugin) decodes the stream back into raw `sensor_msgs/msg/Image` for standard ROS tools.

## Automated Bringup

The video pipeline is integrated into the primary bringup script. It automatically detects the machine type and starts the correct node.

### On both Drone and GCS:
```bash
ros2 launch drone_control racing_bringup.launch.py
```

*   **On Jetson**: Starts the hardware-accelerated publisher.
*   **On Ground Station**: Starts the republisher with the necessary QoS overrides.

## QoS Troubleshooting

The most common issue is an **incompatible QoS** warning. This happens if the subscriber (GCS) expects a "Reliable" connection while the publisher (Jetson) is sending "Best Effort".

### Manual Fix
If you need to run the republisher manually, use the following command to force the QoS connection:

```bash
ros2 run image_transport republish h264 \
  --ros-args \
  --remap in/h264:=/image_raw/h264 \
  --remap out:=/image_raw \
  -p qos_overrides./image_raw/h264.subscription.reliability:=best_effort
```

## Viewing in Foxglove Studio

Foxglove Studio is the recommended way to view the stream directly without needing a local republisher node.

1.  Add a **"Raw Video"** panel.
2.  Select the `/image_raw/h264` topic.
3.  In the panel settings (gear icon), set **Format** to `h264`.
4.  Foxglove will handle the QoS connection automatically.

## Performance Tuning

You can adjust the bitrate and latency in the `jetson_h264_publisher` parameters:

| Parameter | Default | Description |
|---|---|---|
| `bitrate` | 4,000,000 | Target bits per second. |
| `peak_bitrate` | 6,000,000 | Maximum burst bitrate. |
| `iframe_interval` | 15 | Number of frames between IDR frames. |
| `preset_level` | 1 | 1 = UltraFast (lowest latency). |
