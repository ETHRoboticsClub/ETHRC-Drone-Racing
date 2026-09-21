# Vicon — technical

**Owner:** [Jules Schmutz](https://github.com/juliomathis)

Vicon is the motion-capture system in the flight arena and, today, the pose source the controller flies on. Retroreflective markers on the drone are tracked by infrared cameras; the Vicon PC runs Tracker 3 and serves poses over the DataStream SDK. The ROS side is `src/ros2-vicon-receiver` (submodule, fork `ETHRoboticsClub/ros2-vicon-receiver`, pinned at `49a0263`, version 1.0.2).

## Node

`vicon_receiver` connects to `hostname` (Vicon PC), enables segment data, and for every subject/segment pair it sees creates a publisher on the fly.

| Topic | Type | Content |
| :--- | :--- | :--- |
| `/<namespace>/<subject>/<segment>` | `geometry_msgs/PoseStamped` | Pose in `world_frame` (`map`). Position in metres (Vicon millimetres are converted), orientation as quaternion. Created on first sight of the object |
| `/tf` | `TransformStamped` `vicon` → `<subject>_<segment>` | Same pose, every frame |
| `/tf_static` | `TransformStamped` `map` → `vicon` | From `map_xyz` / `map_rpy` at launch |

Parameters (launch arguments of `client.launch.py` have the same names):

| Parameter | Default | Meaning |
| :--- | :--- | :--- |
| `hostname` | `127.0.0.1` (launch: `192.168.10.1`) | Vicon PC. The bringup passes `192.168.0.78` |
| `buffer_size` | 200 | DataStream client buffer |
| `namespace` | `vicon` | Topic prefix |
| `world_frame`, `vicon_frame` | `map`, `vicon` | Frame names |
| `map_xyz`, `map_rpy`, `map_rpy_in_degrees` | zeros, false | Offset of the Vicon origin in `map` |

Header stamps are the receiver node's clock when the frame is processed; the pinned version does not read the SDK latency or compensate for it, and does not filter occlusions.

## Where the pose goes

`racing_bringup.launch.py` builds the topic name from `vicon_subject` and `vicon_segment` (both default to the Jetson hostname) and passes it to `state_bridge_node`, which fuses it with `/imu0` into the controller state at 100 Hz. The same topic is what replay tools convert into a synthetic VIO odometry ([Overview → Workflow](../overview/workflow.md#replay-a-recorded-flight)). The EKF config contains a commented-out `pose1` entry for Vicon; it is not active.

## Frames

Vicon defines `map` by the L-frame placed on the floor during calibration: origin at the L, axes along its arms, z up. The `world` frame used by the EKF and gate detector is a separate name; the replay launch publishes an identity `world`→`map` so the two can be compared. There is no automatic relation between `map` and the OpenVINS `global` frame.

## Files

| Path | Purpose |
| :--- | :--- |
| `src/ros2-vicon-receiver/vicon_receiver/` | Node source and `launch/client.launch.py` |
| `src/drone_control/drone_control/state_bridge_node.py` | Consumer in flight |
| `scripts/perception/replay/vicon_to_odometry.py` | Vicon → `nav_msgs/Odometry` adapter for EKF replays |
| `bags/tools/` | Bag → trajectory JSON conversion using the Vicon topic |

## Known limitations

- Network latency and Tracker processing delay are not compensated; the stamp is arrival time on the Jetson.
- Marker occlusion produces frozen or jumping poses that pass straight through to the controller; `state_bridge_node` only rejects NaN.
- Tracker 4 is not compatible with the MX-F40 cameras; the lab PC must stay on Tracker 3.
