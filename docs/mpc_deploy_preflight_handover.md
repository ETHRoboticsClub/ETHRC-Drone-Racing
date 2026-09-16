# MPC Deploy Preflight Handover

Branch: `feature/mpc_deploy_preflight`

Main workspace:

- Host: `/home/dichau/proj/ETHRC-Drone-Racing`
- Boreas: `/home/aero/proj/ETHRC-Drone-Racing`

## What Changed

### DDS / Runtime Environment

On Boreas, the controller must run with CycloneDDS and ACADOS available. The controller launch now sets ACADOS defaults for launched processes:

```bash
ACADOS_SOURCE_DIR=/home/aero/acados
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/aero/acados/lib
```

Boreas may still have an older overlay at `/home/aero/install`. During deployment, the freshly built `drone_control` and `betaflight_msp_controller` installs were synced there too so stale source order does not hide these changes.

### Coordinate Frames

The controller internals use NED world and FRD body:

- position: north, east, down
- velocity: north, east, down
- quaternion: NED world to FRD body
- angular velocity: FRD body rates

ROS-facing pose goals and Vicon data are treated as ENU/FLU by default and converted at the boundary:

- ENU -> NED: `(north, east, down) = (y, x, -z)`
- FLU -> FRD: `(x, y, z) = (x, -y, -z)`

Relevant launch defaults in `racing_bringup.launch.py`:

```bash
pose_world_frame:=enu
pose_body_frame:=flu
imu_body_frame:=flu
controller_type:=mpc
print_mpc_inputs:=true
```

### MPC Input Logging

`drone_controller_node.py` supports:

```bash
print_mpc_inputs:=true
mpc_input_debug_interval_sec:=1.0
```

The log prints the current MPC state, goal state, position error, and whether state has been received.

### Pose Goal Client

`send_pose_goal.py` sends ROS ENU goals:

```bash
ros2 run drone_control send_pose_goal goal_only -x EAST -y NORTH -z UP -yaw YAW_DEG
```

The existing `z - 0.15` bias is intentional and retained.

Ctrl+C now sends a cancel request and spins briefly so the action server clears its active-goal state. This prevents the next goal from being rejected after an interrupted client.

### MSP / Betaflight Sign Mapping

The MPC publishes semantic body rates to `/drone_attitude_cmd`:

```text
[roll_deg_s, pitch_deg_s, yaw_deg_s, thrust]
```

The MSP bridge maps those rates to Betaflight RC channels. Based on no-prop hand testing, roll and pitch must be inverted at this boundary:

```bash
roll_rate_sign: -1.0
pitch_rate_sign: -1.0
yaw_rate_sign: 1.0
```

These can be overridden from launch:

```bash
ros2 launch drone_control racing_bringup.launch.py \
  roll_rate_sign:=-1.0 pitch_rate_sign:=-1.0 yaw_rate_sign:=1.0
```

## Standard Bringup

Use the repo install, not the stale top-level install:

```bash
cd ~/proj/ETHRC-Drone-Racing
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch drone_control racing_bringup.launch.py \
  controller_type:=mpc \
  vicon_subject:=boreas \
  vicon_segment:=boreas
```

Check that the action server is alive:

```bash
ros2 action info /drone_set_pose_goal
```

Expected:

```text
Action servers: 1
    /drone_controller_node
```

## No-Prop Preflight Test

Command:

```bash
ros2 run drone_control send_pose_goal goal_only -x 0 -y 0 -z 1.5 -yaw 0
```

Inspect both:

```bash
ros2 topic echo /drone_attitude_cmd
```

and the MSP bridge log:

```text
Sending MSP RC: R:... P:... T:... Y:...
```

Expected with goal at origin:

- If drone is at negative NED x, `/drone_attitude_cmd` pitch may be negative, but MSP `P` should go above `1500`.
- If drone is at positive NED x, MSP `P` should go below `1500`.
- If drone is at positive NED y, `/drone_attitude_cmd` roll may be negative, but MSP `R` should go above `1500`.
- If drone is at negative NED y, MSP `R` should go below `1500`.

The body-rate topic is pre-RC-conversion; the MSP log is the final Betaflight-facing sign check.

## Boreas Deployment Notes

Backups created on Boreas during this session:

```text
/home/aero/coord_fix_backups/drone_control_coord_fix_20260630_124403.tar.gz
/home/aero/coord_fix_backups/drone_control_home_install_20260630_133254.tar.gz
/home/aero/coord_fix_backups/rate_sign_home_install_20260630_141017.tar.gz
```

Build commands used:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select drone_control betaflight_msp_controller
```

## Files In This Checkpoint

- `src/drone_control/drone_control/drone_controller_node.py`
- `src/drone_control/drone_control/state_bridge_node.py`
- `src/drone_control/drone_control/send_pose_goal.py`
- `src/drone_control/drone_control/send_trajectory_goal.py`
- `src/drone_control/drone_control/waypoint_visualizer.py`
- `src/drone_control/launch/racing_bringup.launch.py`
- `src/drone_control/launch/drone_controller.launch.py`
- `src/drone_control/setup.py`
- `src/betaflight_msp_controller/betaflight_msp_controller/msp_bridge.py`
- `src/betaflight_msp_controller/config/msp_params.yaml`
- `src/betaflight_msp_controller/launch/msp_bridge.launch.py`
