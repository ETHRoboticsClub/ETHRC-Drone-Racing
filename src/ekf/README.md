# EKF Package (ROS 2)

This package hosts the Extended Kalman Filter (EKF) work for the project.

- Future custom EKF logic will live in `src/nodes/ekf_node.cpp`, migrating core filtering from `src/ekf.cpp`.
- For now, use the Robot Localization (RL) package’s EKF via the included launch file, instead of running our `ekf_node`.

## Launch (Robot Localization EKF)

Use the provided launch file to start RL’s `ekf_node` with this package’s parameters.

1) Build the package (from the workspace root):

```bash
colcon build --packages-select ekf
```

2) Source your workspace:

```bash
source install/setup.bash
```

3) Launch the EKF:

```bash
ros2 launch ekf rl_ekf.launch.xml
```

- This starts `robot_localization`’s `ekf_node` named `ekf_filter_node`.
- Parameters are loaded from `config/rl_ekf.yaml` in this package.
- Fused odometry is published on `/ekf/odometry` (remap from `odometry/filtered`).

## Quick Testing

An optional mock publisher is provided to feed example messages that match the RL config topics.

- In a separate terminal (after sourcing):

```bash
ros2 run ekf mock_publisher
```

- Verify output topics and messages:

```bash
ros2 topic list
ros2 topic echo /ekf/odometry
```

## Configuration

Tweak `config/rl_ekf.yaml` to fit the setup (frames, frequency, and input topics). The default configuration fuses:

- `/vio/odometry` as an Odometry source
- `/vision/pose` as an absolute pose fix

We use command inputs on `cmd_vel` (currently remapped to `control/command`) to estimate the acceleration and optimize the estimate.

## TODOs for full pipeline connection

- Remap the VIO, vision, and control topics in `launch/rl_ekf.launch.xml`, and keep `odom0`/`pose0`/`control0` in `config/rl_ekf.yaml` in sync with the real ROS 2 names.
- Verify that for each sensor, frames and headers (frame ids) of topics match `base_link`/`odom` so Robot Localization can resolve it.
- Verify message types and covariance: VIO should publish `nav_msgs/Odometry` with nonzero covariance; vision should output `PoseWithCovarianceStamped` in the same frame.
- Tune timing for real rates: adjust `frequency`, `sensor_timeout`, `history_length`, and the `odom0_`/`pose0_` queue sizes or `nodelay` settings to match VIO/vision rates and delays.
- Bring up without controls first: set `use_control` to false until the controller publishes stable commands; when enabling, remap `cmd_vel` to the real control topic, set `stamped_control` to match its message type, and tune `control_config`/`control_timeout`.
- Remap the fused output (`odometry/filtered`) to the topic expected by downstream consumers (flight stack, mapper, logger) and validate by bag replay before flight.

## Notes

- Do not launch `ekf_node` (our custom node) yet—it’s a placeholder for future EKF implementation.
