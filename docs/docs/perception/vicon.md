# Vicon Motion Capture

**Page Owner:** [Jules Schmutz](https://github.com/juliomathis)
---

## Overview

The Vicon system provides accurate 6-DOF pose estimates (position and orientation) by tracking retroreflective markers attached to the drone using an array of infrared cameras. It is used as ground truth localization for indoor flights.

The `ros2-vicon-receiver` package connects to the Vicon DataStream SDK server running on the Vicon PC, retrieves tracking data at the camera frame rate, and publishes it as ROS 2 topics.

## Output Topics

For each tracked object (subject + segment pair defined in Vicon Tracker), the node automatically creates a publisher:

### Pose
- **Topic**: `/vicon/<SubjectName>/<SegmentName>`
- **Type**: `geometry_msgs/msg/PoseStamped`
- **Description**: 6-DOF pose of the tracked object expressed in the world (`map`) frame. Position is in meters; orientation is a quaternion.
- **Example**: `/vicon/drone/drone`

### Transforms (TF)
- **`/tf`** — Dynamic transform from the `vicon` frame to each `<SubjectName>_<SegmentName>` frame, broadcast every frame.
- **`/tf_static`** — Static transform from `map` to `vicon` origin, set at launch via `map_xyz` and `map_rpy` parameters.

## Launch

```bash
ros2 launch vicon_receiver client.launch.py \
  hostname:=192.168.0.29 \
  topic_namespace:=vicon \
  buffer_size:=200 \
  world_frame:=map \
  vicon_frame:=vicon \
  map_xyz:='[0.0, 0.0, 0.0]' \
  map_rpy:='[0.0, 0.0, 0.0]' \
  map_rpy_in_degrees:=false
```

### Parameters

| Parameter | Default | Description |
|---|---|---|
| `hostname` | `127.0.0.1` | IP address of the Vicon PC running the DataStream server |
| `buffer_size` | `200` | DataStream SDK internal buffer size |
| `world_frame` | `map` | Name of the world frame in TF |
| `vicon_frame` | `vicon` | Name of the Vicon origin frame in TF |
| `map_xyz` | `[0,0,0]` | Translation (meters) from `world_frame` to `vicon` origin |
| `map_rpy` | `[0,0,0]` | Rotation (rad or deg) from `world_frame` to `vicon` origin |
| `map_rpy_in_degrees` | `false` | Set to `true` if `map_rpy` values are in degrees |

## Setting Up a Tracked Object

In **Vicon Tracker** software on the Vicon PC:

1. Create a new object in Vicon Tracker, assign the markers, and give it a name (e.g. `drone`).
3. The segment name defaults to the same as the subject name.
4. The resulting ROS topic will be `/vicon/drone/drone`.
