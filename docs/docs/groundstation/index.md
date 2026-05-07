# Groundstation

The ground station is responsible for operator visibility, telemetry
inspection, and offline analysis. For the racing camera path, the drone publishes
an H.264 ROS topic and the ground station decodes it back into a normal image
topic for viewing.

## Video Topics

| Topic | Meaning | View Directly? |
| --- | --- | --- |
| `/cam0/image_h264` | H.264 bytes in `sensor_msgs/CompressedImage` | No |
| `/image_decoded` | Decoded image from `isaac_ros_h264_decoder` | Yes |
| `/cam0/image_raw` | Mono perception image on the drone | Only for debug |
| `/cam0/image_rgb` | Encoder input branch on the drone | No |

Start with [H.264 Video Streaming](video-streaming.md) when setting up live
video or bagging camera logs.
