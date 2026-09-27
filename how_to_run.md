# How to Run the Simulation

> Run all commands **inside the Docker container** unless stated otherwise.
> In every new terminal, source the workspace first:
> ```bash
> source ~/ros_ws/install/setup.bash
> ```

---

## Default — ROS-only MPCC++ Simulator

The default launch now uses the lightweight simulator built from the exact
`mpcc_pp` dynamics. PX4 and Gazebo are not required for this path.

```bash
cd ~/ros_ws
source install/setup.bash
ros2 launch drone_control sim.launch.py own_simulator:=true use_mavsdk:=false
```

This starts:

| Node | Role |
|---|---|
| `mpcc_pp_dynamics_simulator` | Integrates the 19-state MPCC++ model and publishes `drone_state_sim` plus `/mpcc_pp/sim_pose` |
| `drone_controller_node` | Runs the selected controller and publishes raw MPCC++ actions to `/mpcc_pp/mpc_action` |
| visualizer/RViz | Shows gates, path, horizon, and simulator pose |

Useful options:

| Argument | Values | Default |
|---|---|---|
| `controller_type` | `mpc`, `mpcc`, `mpcc_pp`, `mpc_rotor`, `position_ned`, `rl` | `mpcc_pp` |
| `waypoints_file` | path to `.yaml` | `gates_config.yaml` |
| `own_simulator` | `true` / `false` | `false` |
| `use_mavsdk` | `true` / `false` | `true` |
| `rviz` | `true` / `false` | `true` |
| `plotjuggler` | `true` / `false` | `false` |

Example:
```bash
ros2 launch drone_control sim.launch.py controller_type:=mpcc_pp rviz:=true
```

---

## Legacy — PX4 SITL + Gazebo

Keep using this only when you explicitly want the old PX4/Gazebo pipeline.

### Terminal 1 — PX4 SITL + Gazebo

```bash
cd ~/ros_ws
source scripts/source_sitl_env.sh
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
cd ~/ros_ws
source scripts/source_sitl_env.sh

# Required for PX4's uXRCE-DDS client. The run_sim.sh wrapper starts this
# automatically; start it yourself when invoking make directly.
MicroXRCEAgent udp4 -p 8888 >/tmp/micro_xrce_agent_8888.log 2>&1 &

cd ~/ros_ws/px4/PX4-Autopilot
PX4_GZ_WORLD=default make px4_sitl gz_zephyros HEADLESS=1
```

Wait until you see `Ready for takeoff!` in the PX4 output before proceeding.

### Terminal 2 — ROS 2 Controller Against PX4

```bash
cd ~/ros_ws
source scripts/source_sitl_env.sh

# Existing containers may use the user-local v0.2.1 renderer. Fresh images
# use the renderer installed under /opt/acados by docker/Dockerfile.
if [ -x "$HOME/.local/bin/t_renderer" ]; then
  export TERA_PATH="$HOME/.local/bin/t_renderer"
fi

ros2 launch drone_control sim.launch.py \
  own_simulator:=false \
  use_mavsdk:=true \
  controller_type:=mpcc_pp \
  waypoints_file:="$HOME/ros_ws/src/drone_control/config/gates_config.yaml" \
  mpcc_stabilization_sec:=3.0 \
  acados_source_dir:=/opt/acados \
  acados_lib_dir:=/opt/acados/lib \
  rviz:=true
```

The PX4 path deliberately keeps the 3-second MPC stabilization phase before
MPCC++ takes over. It therefore needs the shared Boreas profile at
`~/ros_ws/config/airframes/boreas_4s.yaml`. The Compose configuration mounts
that directory read-only; recreate an already-running development container
after pulling this change:

```bash
# Run on the host, from the repository root.
docker compose up -d --build --force-recreate dev
docker compose exec dev bash
```

---

## Optional — Perception Pipeline

Only needed if using camera-based state estimation (ArUco, VIO, EKF):

```bash
source ~/ros_ws/install/setup.bash
ros2 launch launch/perception.launch.py
```

---

## Startup Order

ROS-only default:

```
Terminal 1: ROS 2 launch  -> starts simulator + controller + visualization
```

Legacy PX4/Gazebo:

```
Terminal 1: PX4 SITL + Gazebo   -> wait for "Ready for takeoff!"
Terminal 2: ROS 2 Controller    -> starts controller + MAVSDK + RViz
Terminal 3: Perception (opt.)   -> starts bridge + ArUco + EKF
```
