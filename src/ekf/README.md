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

## Notes

- Do not launch `ekf_node` (our custom node) yet—it’s a placeholder for future EKF implementation.
