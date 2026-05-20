# How to Run the Simulation

> Run all commands **inside the Docker container** unless stated otherwise.
> In every new terminal, source the workspace first:
> ```bash
> source ~/ros_ws/install/setup.bash
> ```

---

## Terminal 1 — PX4 SITL + Gazebo

```bash
cd ~/ros_ws
./scripts/run_sim.sh
```

**Options:**

| Command | Effect |
|---|---|
| `./scripts/run_sim.sh` | Zephyros model, real-time speed (1x) |
| `./scripts/run_sim.sh gz_zephyros default_slow` | Zephyros model, half speed (0.5x) |
| `./scripts/run_sim.sh gz_x500_mono_cam default` | x500 mono cam model, real-time speed |

Or directly with `make`:
```bash
cd ~/ros_ws/px4/PX4-Autopilot
PX4_GZ_WORLD=default make px4_sitl gz_zephyros
```

Wait until you see `Ready for takeoff!` in the PX4 output before proceeding.

---

## Terminal 2 — ROS 2 Controller

```bash
source ~/ros_ws/install/setup.bash
ros2 launch drone_control sim.launch.py
```

**Options:**

| Argument | Values | Default |
|---|---|---|
| `controller_type` | `mpc`, `mpcc`, `mpcc_pp`, `mpc_rotor`, `position_ned`, `rl` | `mpcc_pp` |
| `waypoints_file` | path to `.yaml` | `gates_config.yaml` |
| `rviz` | `true` / `false` | `true` |
| `plotjuggler` | `true` / `false` | `false` |

Example with custom controller:
```bash
ros2 launch drone_control sim.launch.py controller_type:=mpc
```

---

## (Optional) Terminal 3 — Perception Pipeline

Only needed if using camera-based state estimation (ArUco, VIO, EKF):

```bash
source ~/ros_ws/install/setup.bash
ros2 launch launch/perception.launch.py
```

---

## Startup Order

```
Terminal 1: PX4 SITL + Gazebo   →  wait for "Ready for takeoff!"
Terminal 2: ROS 2 Controller    →  starts controller + MAVSDK + RViz
Terminal 3: Perception (opt.)   →  starts bridge + ArUco + EKF
```
