# Gazebo ROS 2 Bridge

## Overview

The Gazebo ROS 2 bridge package provides a configurable interface to bridge topics between Gazebo Ignition (Gz) and ROS 2. This allows sensor data from Gazebo simulations to be accessible as standard ROS 2 topics, enabling integration with ROS 2-based perception and control systems.

The bridge uses the `ros_gz_bridge` package's `parameter_bridge` executable to convert between Gazebo Ignition message types and ROS 2 message types.

## How It Works

The bridge reads a YAML configuration file that defines which Gazebo topics should be bridged to ROS 2, along with their message types and optional topic remappings. The launch file automatically configures and starts the bridge based on this configuration.

## Current Bridges

The default configuration (`config/gz_bridges.yaml`) bridges the following topics. Both the `drone` and `x500_cam` models publish identical Gazebo topics, so the same bridge works for both without any arguments:

1. **Camera Image**: `/drone/camera/image` (Gz) → `/cam0/image_raw` (ROS 2)
   - Type: `gz.msgs.Image` ↔ `sensor_msgs/msg/Image`

2. **Camera Info**: `/drone/camera/camera_info` (Gz) → `/cam0/camera_info` (ROS 2)
   - Type: `gz.msgs.CameraInfo` ↔ `sensor_msgs/msg/CameraInfo`

3. **IMU**: `/drone/imu` (Gz) → `/imu0` (ROS 2)
   - Type: `gz.msgs.IMU` ↔ `sensor_msgs/msg/Imu`

4. **Ground Truth Odometry**: `/drone/odometry` (Gz) → `/odometry` (ROS 2)
   - Type: `gz.msgs.Odometry` ↔ `nav_msgs/msg/Odometry`

## Launching the Bridge

Works identically for both `gz_drone` and `gz_x500_cam`:

```bash
ros2 launch gazebo_ros_bridges bridge.launch.py
```

To use a custom configuration file:

```bash
ros2 launch gazebo_ros_bridges bridge.launch.py config:=/path/to/your/config.yaml
```

## How to Add a New Bridge

To add a new bridge between Gazebo and ROS 2, follow these steps:

### Step 1: Identify the Gazebo Topic

First, identify the Gazebo topic you want to bridge. You can list all Gazebo topics using:

```bash
gz topic -l
```

Or inspect a specific topic:

```bash
gz topic -i /your/gazebo/topic
```

### Step 2: Determine Message Types

You need to know:
- **Gazebo message type**: Check the topic info or Gazebo documentation (e.g., `gz.msgs.Image`, `gz.msgs.IMU`, `gz.msgs.LaserScan`)
- **ROS 2 message type**: The corresponding ROS 2 message type (e.g., `sensor_msgs/msg/Image`, `sensor_msgs/msg/Imu`, `sensor_msgs/msg/LaserScan`)

Common mappings:
- `gz.msgs.Image` ↔ `sensor_msgs/msg/Image`
- `gz.msgs.IMU` ↔ `sensor_msgs/msg/Imu`
- `gz.msgs.CameraInfo` ↔ `sensor_msgs/msg/CameraInfo`
- `gz.msgs.LaserScan` ↔ `sensor_msgs/msg/LaserScan`
- `gz.msgs.PointCloudPacked` ↔ `sensor_msgs/msg/PointCloud2`

### Step 3: Edit the Configuration File

Open `config/gz_bridges.yaml` and add a new bridge entry:

```yaml
bridges:
  # ... existing bridges ...
  
  - name: your_bridge_name
    gz_topic: "/your/gazebo/topic"
    ros2_type: "sensor_msgs/msg/YourMessageType"
    gz_type: "gz.msgs.YourGzMessageType"
    remap_to: "/your/ros2/topic"  # Optional: remap to a different ROS 2 topic name
    enabled: true
```

### Step 4: Example - Adding a Laser Scan Bridge

Here's a complete example for adding a LiDAR/Laser scan bridge:

```yaml
bridges:
  # ... existing bridges ...
  
  - name: lidar_scan
    gz_topic: "/lidar/scan"
    ros2_type: "sensor_msgs/msg/LaserScan"
    gz_type: "gz.msgs.LaserScan"
    remap_to: "/scan"
    enabled: true
```

### Step 5: Restart the Bridge

After modifying the configuration file, restart the bridge:

```bash
ros2 launch gazebo_ros_bridges bridge.launch.py
```

### Step 6: Verify the Bridge

Check that the ROS 2 topic is available:

```bash
ros2 topic list
ros2 topic echo /your/ros2/topic
```

## Configuration File Structure

The configuration file (`gz_bridges.yaml`) has the following structure:

```yaml
bridges:
  - name: <descriptive_name>          # Human-readable name (optional, for documentation)
    gz_topic: "<gazebo_topic_path>"    # Full path to Gazebo topic
    ros2_type: "<ros2_msg_type>"      # ROS 2 message type (e.g., "sensor_msgs/msg/Image")
    gz_type: "<gz_msg_type>"          # Gazebo message type (e.g., "gz.msgs.Image")
    remap_to: "<ros2_topic_name>"     # Optional: ROS 2 topic name (defaults to gz_topic if not specified)
    enabled: true                     # Optional: Enable/disable this bridge (default: true)
```

## Disabling a Bridge

To temporarily disable a bridge without removing it from the configuration, set `enabled: false`:

```yaml
  - name: camera_image
    gz_topic: "/drone/camera/image"
    ros2_type: "sensor_msgs/msg/Image"
    gz_type: "gz.msgs.Image"
    remap_to: "/cam0/image_raw"
    enabled: false  # Bridge is disabled
```

## Troubleshooting

### Bridge Not Working

1. **Check Gazebo topic exists**: Verify the Gazebo topic is being published:
   ```bash
   gz topic -l | grep your_topic
   ```

2. **Check message types**: Ensure the message types are compatible. Some conversions may not be supported.

3. **Check ROS 2 topic**: Verify the ROS 2 topic appears:
   ```bash
   ros2 topic list
   ros2 topic info /your/ros2/topic
   ```

4. **Check bridge logs**: The launch file prints configuration information. Look for errors in the console output.

### Topic Not Remapping

If the remapping doesn't work, check:
- The `remap_to` field is correctly specified
- The launch file is using the correct configuration file
- No other node is already using the target topic name

## Notes

- The bridge is unidirectional by default (Gazebo → ROS 2). For bidirectional bridges, you may need to configure additional parameters.
- Some message type conversions may require custom conversion code if not natively supported by `ros_gz_bridge`.
- The bridge maintains the message frequency from the source (Gazebo) topic.

