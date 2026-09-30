# MPC Deploy Preflight: History and Current State

This page records what the MPC deploy preflight commits contained, why they were made, and what has happened to that code after the September 2026 merges into `main`.

The original handover note is [`mpc_deploy_preflight_handover.md`](../mpc_deploy_preflight_handover.md). It describes the code as it was on 30 June 2026. **Much of it no longer matches `main`**; use this page to find where things went.

## TL;DR

- Both commits are fully merged into `main`, so the branches `feature/mpc_deploy_preflight` (and the misspelled local `feature/mpc_deploy_prefligth`) hold nothing that is not already on `main`.
- The preflight MPC controller node **was renamed**. The action-server node that had been `drone_controller_node.py` is now `racing_controller_node.py`, launched by `racing_controller.launch.py`. On `main`, `drone_controller_node.py` is a different, much smaller node from the RL deploy line.
- The preflight versions of `racing_bringup.launch.py` and `state_bridge_node.py` **were dropped** in favour of `main`'s RL-deploy versions. The ENU→NED conversion in the state bridge and the frame and sign launch arguments in the bringup no longer exist.
- The MSP rate-sign inversion (`roll_rate_sign = pitch_rate_sign = -1`) **survived**. `DroneMpcController` now *also* negates pitch. The net pitch sign seen by Betaflight is therefore flipped compared with the June no-prop hand test. See [Rate-sign stacking](#rate-sign-stacking-must-re-check-before-flight).
- `send_pose_goal.py`, `waypoint_visualizer.py`, `msp_params.yaml` and `msp_bridge.launch.py` are byte-identical to the preflight commit.

## The original commits

| Commit | Date | Author | Branch tip of |
|---|---|---|---|
| `81e4a2bc0` | 29 Jun 2026 | Emilio Soltero | `feature/mpc_deploy` |
| `aa3a3a975` | 30 Jun 2026 | Dimetri Chau | `feature/mpc_deploy_preflight` (created on top of `81e4a2bc0`) |

### `81e4a2bc0`: Added state_bridge_node and racing_bringup.launch.py

**Why:** the MPC branch (`feature/mpc_deploy`) had no hardware bringup. To fly MPC on Boreas with Vicon, it borrowed the hardware pipeline already built on `feature/rl_deploy`.

| File | Change |
|---|---|
| `src/drone_control/drone_control/state_bridge_node.py` | New (116 lines). Fuses Vicon pose and `/imu0` rates into a 13-float `Float64MultiArray` state, bypassing the EKF. |
| `src/drone_control/launch/racing_bringup.launch.py` | New (513 lines). Full hardware bringup: IMU, MSP bridge, Vicon, camera, state bridge, controller. |
| `src/drone_control/config/drone_controller_config.yaml` | Added `t_horizon: 5.0` and `n_nodes: 50`. |

### `aa3a3a975`: Prepare MPC deploy preflight checkpoint

**Why:** to get the MPC ready for a no-prop preflight on Boreas. The controller had been fed Vicon data with no frame conversion, the Betaflight rate signs were wrong, and debugging was hard. The details and test procedure are in the [handover note](../mpc_deploy_preflight_handover.md). In summary:

| Area | Change | Files |
|---|---|---|
| Coordinate frames | Controller internals fixed to NED world / FRD body. ROS-facing pose goals and Vicon data treated as ENU/FLU and converted at the boundary (`ENU→NED: (y, x, -z)`, `FLU→FRD: (x, -y, -z)`). PoseStamped state and goals go through `_pose_stamped_to_ned_frd_state()`. | `drone_controller_node.py`, `state_bridge_node.py`, `waypoint_visualizer.py` |
| State bridge | New params `pose_world_frame`, `pose_body_frame`, `imu_body_frame` (default `enu`/`flu`/`flu`). Output quaternion is `wxyz` NED/FRD. IMU subscription set to BEST_EFFORT QoS. Vicon topic defaults to the hostname (`boreas`/`zephyrus`). | `state_bridge_node.py` |
| MPC input logging | New params `print_mpc_inputs` and `mpc_input_debug_interval_sec`. `_log_mpc_inputs()` prints state, goal, position error and `state_received` at a throttled rate. | `drone_controller_node.py`, `drone_controller.launch.py` |
| Import paths | `sys.path` resolution no longer hard-codes `~/ros_ws/src`. It searches the source tree, the install tree, `~/proj/ETHRC-Drone-Racing/src`, then `~/ros_ws/src`. | `drone_controller_node.py` |
| ACADOS env | Controller launch sets `ACADOS_SOURCE_DIR=/home/aero/acados` and appends `/home/aero/acados/lib` to `LD_LIBRARY_PATH`. | `drone_controller.launch.py` |
| MSP sign mapping | `msp_bridge` multiplies body rates by `roll_rate_sign`, `pitch_rate_sign`, `yaw_rate_sign` (defaults `-1`, `-1`, `+1`, based on no-prop hand testing). These can be overridden from launch. | `msp_bridge.py`, `msp_params.yaml`, `msp_bridge.launch.py`, `racing_bringup.launch.py` |
| Pose goal client | Sends ROS ENU goals and keeps the intentional `z - 0.15` bias. Ctrl+C sends a cancel and spins briefly so the server clears its active goal. | `send_pose_goal.py` |
| Trajectory client | Waypoints file resolved from the package share directory. | `send_trajectory_goal.py` |
| Bringup defaults | `controller_type:=mpc`, `print_mpc_inputs:=true`, frame and sign args wired to the state bridge, MSP bridge and controller. | `racing_bringup.launch.py` |
| Packaging | `state_bridge_node` entry point added. | `setup.py` |
| Docs | Handover note added. | `docs/mpc_deploy_preflight_handover.md` |

## How the code reached `main`

```text
aa3a3a975 (feature/mpc_deploy_preflight)
   │  PR #83, merged 16 Sep 2026 (5230a297e)
   ▼
feature/mpc_deploy
   │  87fc93b6c, 18 Sep 2026
   ▼
feature/mpc-main
   │  pre-merge cleanup cf9bec062, 20 Sep 2026
   │  "Merge main into integrate/mpc" acb005bff, 20 Sep 2026  ◄── conflicts resolved here
   ▼
integrate/mpc
   │  6413360da "Merge integrate/mpc: MPCC/MPCC++ racing stack", 20 Sep 2026
   ▼
main
   │  45f02133d  MPCC++ residual modelling merge (20 Sep)
   │  79a0975cd  PR #86: restore MPCC++ startup in sims (21 Sep)
   │  6952d3c9a  PR #87: MPCC++ dependency compatibility, incl. MPC state-passing fix (27 Sep)
```

Almost every structural change happened in `acb005bff`. `main` had developed its own RL deploy stack in parallel, including an RL-oriented `drone_controller_node.py`, `racing_bringup.launch.py` and `state_bridge_node.py`. The resolution was:

- **Keep `main`'s files** under their original names.
- **Rename the MPC branch's node and launch file**, keeping them next to `main`'s.

## Where each file is now

Compared with `aa3a3a975` on current `main` (`6952d3c9a`). The `docs/mpc` branch has no differences from `main` in these files.

| Preflight file | Status on `main` | Where the preflight code is now |
|---|---|---|
| `drone_control/drone_controller_node.py` | **Replaced.** Now `main`'s 305-line RL-deploy node (topic-based, no action servers). | `drone_control/racing_controller_node.py` (1468 lines) |
| `launch/drone_controller.launch.py` | **Replaced.** Now `main`'s RL-deploy launch file for `drone_controller_node`. | `launch/racing_controller.launch.py` |
| `launch/racing_bringup.launch.py` | **Replaced.** Identical to `main` before the MPC merge (RL-deploy version, default `controller_type:=rl`). | Gone. None of the preflight bringup args survive. |
| `drone_control/state_bridge_node.py` | **Replaced.** `main`'s RL-deploy version (Vicon or Gazebo odometry, ENU/FLU, `xyzw`). | Gone. No frame conversion remains in the bridge. |
| `betaflight_msp_controller/msp_bridge.py` | Kept, with additions from `main` | Same file. The sign parameters are intact. |
| `config/msp_params.yaml` | Unchanged | Same file |
| `launch/msp_bridge.launch.py` | Unchanged | Same file |
| `drone_control/send_pose_goal.py` | Unchanged | Same file |
| `drone_control/waypoint_visualizer.py` | Unchanged | Same file |
| `drone_control/send_trajectory_goal.py` | Cosmetic only (`warn` → `warning`) | Same file |
| `config/drone_controller_config.yaml` | `controller_rate` is now 50 Hz (was 100 Hz), from `feature/mpc_turbo` (`0f9da693c`) | Same file |
| `setup.py` | Entry points added for `racing_controller_node`, `aigp_characterization`, `teleop_keyboard_node`, `estimator_visualizer_node` | Same file |
| `docs/mpc_deploy_preflight_handover.md` | Unchanged on `main`. Moved to `docs/docs/` on `docs/mpc` (`84218bfc1`) | `docs/docs/mpc_deploy_preflight_handover.md` |

### Preflight features that still exist

All of these now live in `racing_controller_node.py` and `racing_controller.launch.py`:

- `_ENU_TO_NED`, `_FLU_TO_FRD`, quaternion helpers, `_pose_frame_is_ned()` and `_pose_stamped_to_ned_frd_state()`. PoseStamped states and goals are still converted ENU/FLU → NED/FRD (or passed through if `frame_id` is `ned`, `map_ned` or `world_ned`).
- `print_mpc_inputs` and `mpc_input_debug_interval_sec`, as launch args of `racing_controller.launch.py`. **Default is now `false`.** The preflight bringup defaulted it to `true`.
- The `sys.path` resolution block.
- ACADOS env setup, now exposed as `acados_source_dir` and `acados_lib_dir` launch args (defaults `/home/aero/acados` and `/home/aero/acados/lib`).
- The `drone_set_pose_goal` and `drone_set_trajectory_goal` action servers, which `send_pose_goal` and `send_trajectory_goal` talk to.
- `roll_rate_sign`, `pitch_rate_sign`, `yaw_rate_sign` in `msp_bridge` (defaults `-1`, `-1`, `+1`).

### Preflight features that are gone

- `state_bridge_node` parameters `pose_world_frame`, `pose_body_frame` and `imu_body_frame`, and the conversion to NED/FRD `wxyz`. The bridge now publishes raw ENU/FLU with an `xyzw` quaternion.
- `racing_bringup.launch.py` args `pose_world_frame`, `pose_body_frame`, `imu_body_frame`, `roll_rate_sign`, `pitch_rate_sign`, `yaw_rate_sign`, `print_mpc_inputs`, `mpc_input_debug_interval_sec`, and the `controller_type:=mpc` default. Passing any of these to `racing_bringup` now has no effect. The MSP bridge still gets its sign defaults from its own parameter declarations.

## Behavior changes

### 1. `racing_bringup.launch.py controller_type:=mpc` runs a different node

On `main`, `racing_bringup` includes `drone_controller.launch.py`, which starts the **RL-deploy `drone_controller_node`**, not the preflight action-server node. With `controller_type:=mpc`, that node:

- subscribes to `drone_state_sim` (`Float64MultiArray`, ENU/FLU, `xyzw`) from `state_bridge_node`;
- takes goals as `geometry_msgs/Point` on `/drone_goal` (ENU, sticky, rejected outside `goal_bounds`), **not** via the `drone_set_pose_goal` action;
- stays silent (publishes nothing) until it has both a goal and a fresh state (< `state_timeout_sec`, 0.1 s);
- rate-limits thrust changes to `max_thrust_delta_per_tick` (0.05).

As a result, the handover's standard bringup plus `ros2 run drone_control send_pose_goal ...` **no longer works**: `ros2 action info /drone_set_pose_goal` will show no server. Send goals with, for example:

```bash
ros2 topic pub --once /drone_goal geometry_msgs/msg/Point "{x: 0.0, y: 0.0, z: 1.5}"
```

The preflight node and its action servers are only started by `racing_controller.launch.py`, which `sim.launch.py` includes. No hardware bringup on `main` launches it.

### 2. The MPC API changed: ENU/FLU `xyzw` in, position-only goal

At the preflight commit, `DroneMpcController.controller_update(x, goal)` took the NED/FRD `wxyz` state and a full 13-element goal state directly. On `main` (from `main`'s "pre-switch" commit `ed2bf04af`), it takes:

- `x_current`: ENU/FLU state with an `xyzw` quaternion, converted to NED/FRD internally;
- `goal_position`: a 3-vector ENU position. The reference attitude is set to the **current** attitude.

Consequences:

- **Yaw goals are ignored by the standard MPC.** `send_pose_goal -yaw ...` still sends an orientation, but it no longer shapes the MPC reference.
- Between the merge (20 Sep) and PR #87 (27 Sep), `racing_controller_node` passed its NED/FRD state straight into this ENU API, so standard MPC in the racing node was wrong in that window. Commit `546790b59` (in PR #87) fixed it. The node now converts back with `_ned_frd_state_to_mpc_enu_flu()` before calling the standard MPC, and handles MAVSDK's `xyzw` layout with `_mavsdk_state_to_ned_frd_state()`. MPCC and MPCC++ still receive NED/FRD directly.

### 3. `racing_controller_node` expects NED/FRD on `Float64MultiArray`

If `racing_controller_node` gets a 13-element `Float64MultiArray` state with `use_mavsdk:=false`, it treats the data as NED/FRD `wxyz` and copies it unchanged. That matched the preflight state bridge. It does **not** match the current `state_bridge_node`, which publishes ENU/FLU `xyzw`. To drive `racing_controller_node` from Vicon on hardware, use `state_msg_type:=PoseStamped` with the Vicon topic as `state_topic`, which still goes through the ENU→NED conversion. Don't use the current state bridge for this. That path has not been tested on hardware since the merge.

### 4. Rate-sign stacking: must re-check before flight

Two independent sign fixes were combined in the merge:

| Source | Roll | Pitch |
|---|---|---|
| Preflight `msp_bridge` (`aa3a3a975`) | × −1 | × −1 |
| `main`'s `DroneMpcController.attitude_cmd_from_mpc_cmd` (`ed2bf04af`) | × +1 | × −1 |

Net sign from FRD body rate to Betaflight RC stick:

| Configuration | Roll | Pitch |
|---|---|---|
| Preflight (June hand test) | −wx | −wy |
| `main` before the MPC merge | +wx | −wy |
| **`main` now** | **−wx** | **+wy** |

The `main` pitch is the opposite of the June hand test. The `main` roll is the opposite of what `main`'s own MPC docs describe ([MPC](mpc.md#summary): "pitch is inverted … roll and yaw retain their FRD signs"). The `msp_bridge` inversion also applies to every other controller publishing on `/drone_attitude_cmd`, including RL.

Before any hardware test, repeat the no-prop check from the [handover note](../mpc_deploy_preflight_handover.md#no-prop-preflight-test). Then set either the MSP bridge signs or the controller adapter so that the correction is applied once. `racing_bringup` no longer exposes the sign args, so override them on the `msp_bridge` node's parameters or change the defaults.

### 5. `msp_bridge` now gates on the override switch

`attitude_cmd_callback` returns early unless the AUX2 override switch (`override_switch_id`, default 5) is on. In the preflight version, `/drone_attitude_cmd` was always mapped to RC channels. With the switch off, joystick axes are passed through as manual RC instead. `msp_bridge` also accepts a Joy fallback as JSON over UDP on `127.0.0.1:9010` (`joy_udp_port`).

During a no-prop test, the `Sending MSP RC: ...` log will show 1500 on R/P/Y until the override switch is on.

### 6. DDS middleware default

`msp_bridge` now calls `configure_local_ros_defaults()`, which does `setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")`. The handover requires CycloneDDS on Boreas, so export `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` in the shell or `.bashrc` before launching. If it isn't set, `msp_bridge` will quietly use Fast DDS. Set `ETHRC_DISABLE_LOCAL_ROS_DEFAULTS=1` to turn this off entirely.

### 7. Other defaults

- `drone_controller_config.yaml` `controller_rate` is 50 Hz (was 100 Hz). This applies to `racing_controller_node`, which reads this file. The RL-deploy `drone_controller_node` gets its rate from the same file.
- `racing_bringup` `vicon_subject` and `vicon_segment` default to the machine's hostname for any host, not just `boreas` and `zephyrus`.

## Branch cleanup

`feature/mpc_deploy_preflight` (local and `origin`), `feature/mpc_deploy` and the local typo branch `feature/mpc_deploy_prefligth` are all ancestors of `main`. You can delete them without losing work:

```bash
git branch -d feature/mpc_deploy_preflight feature/mpc_deploy_prefligth
git push origin --delete feature/mpc_deploy_preflight
```
