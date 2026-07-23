# SITL Simulation & Verification

**Page Owner:** [Leads of the Aeronautics Division](https://github.com/orgs/ETHRoboticsClub/teams/aeronautics/)

---

Software-in-the-Loop (SITL) simulation allows you to test and verify the entire autonomous flight control stack, including neural network RL policies and perception estimators, without risking physical drone hardware.

The SITL environment integrates **Gazebo Harmonic** (for physical world simulation and sensors) with the **Betaflight SITL firmware** (running the same flight-control loops as the real drone) in a closed-loop setup.

---

## 🏗️ Architecture Overview

The simulation mirrors the real-world drone architecture by replacing hardware interfaces with UDP sockets and ROS 2 bridges:

```mermaid
graph TD
    subgraph "Gazebo Harmonic (Physics & Sensors)"
        GZ[gz-sim World] -->|fdm_packet UDP:9003| BF[BF SITL Binary]
        BF -->|servo_packet UDP:9002| GZ
        GZ -->|ros_gz_bridge| ODOM["/odometry (Odometry)"]
        GZ -->|ros_gz_bridge| IMU0["/imu0 (Imu)"]
    end

    subgraph "ROS 2 Companion Computer Stack"
        ODOM --> SB[state_bridge_node]
        IMU0 --> SB
        SB -->|drone_state_sim| DC[drone_controller_node]
        GOAL["/drone_goal (Point)"] --> DC
        DC -->|drone_attitude_cmd| UDP_RC[sitl_rc_bridge]
    end

    UDP_RC -->|rc_packet UDP:9004| BF
```

* **Gazebo** simulates the `drone_sitl` body dynamics, IMU, and odometry. The
  in-tree `EthrcBetaflightSITLPlugin` loads the shared Boreas 4S profile and
  applies identified-style motor lag, thrust, reaction torque, and body drag.
* **Betaflight SITL** receives simulated FDM packets from Gazebo on UDP port `9003`, reads RC input on UDP port `9004`, and sends motor outputs back to Gazebo on UDP port `9002`.
* **`sitl_rc_bridge`** bridges ROS 2 `/joy` and RL or MPC controller commands
  into Betaflight SITL RC packets.

The model profile is `config/airframes/boreas_4s.yaml`. Its rigid-body values
are authoritative, while propulsion and drag remain provisional until physical
identification is complete.

---

## 🛠️ Setup & Build Instructions

Before running the simulation, you must install the Gazebo ROS bridges and compile the simulation plugins and flight firmware.

### 1. Full Bootstrap

```bash
# From the workspace root
./scripts/setup_sitl.sh
```

This script will:

1. Install **Gazebo Harmonic** and the matching ROS/Gazebo bridge packages.
2. Clone or update `ext/aeroloop_gazebo` for reference dependencies and build the in-tree SITL Gazebo plugin.
3. Clone or update `ext/anemos-betaflight` and build the Betaflight SITL target.
4. Load `docs/docs/files/betaflight_conf/sitl_config.txt` into `eeprom.bin`.
5. Run `colcon build --symlink-install --packages-up-to drone_control betaflight_msp_controller gazebo_ros_bridges sitl_gazebo_plugins`.

!!! note
    `setup_sitl.sh` fetches external repositories. On a development branch with local edits inside `ext/aeroloop_gazebo` or `ext/anemos-betaflight`, use the manual rebuild commands below instead of re-running the bootstrap script.

### 2. Manual Rebuild After Local Changes

Use this when dependencies are already installed and you only need to rebuild the checked-out source:

```bash
# Gazebo Betaflight plugin
ext/aeroloop_gazebo/build_plugin.sh

# Betaflight SITL firmware
cd ext/anemos-betaflight
make TARGET=SITL -j$(nproc)
cd ../..

# Regenerate eeprom.bin from the SITL Betaflight CLI config
ext/anemos-betaflight/obj/main/betaflight_SITL.elf \
  --config docs/docs/files/betaflight_conf/sitl_config.txt

# ROS packages
source scripts/source_sitl_env.sh
colcon build --symlink-install \
  --packages-up-to drone_control betaflight_msp_controller gazebo_ros_bridges
```

### 3. Preflight Port Check

Before launching, make sure no previous SITL/Gazebo process is still holding the UDP ports:

```bash
ss -ulpn | rg '900[234]' || true
```

Expected output before launch is empty. If a previous run is still listed, stop it with `Ctrl-C` in that terminal or kill only the stale SITL/Gazebo processes:

```bash
pkill -f 'gz sim .*sitl_racing' || true
pkill -f 'gz-sim-main .*sitl_racing' || true
pkill -f 'betaflight_SITL' || true
pkill -f 'sitl_rc_bridge' || true
```

Re-run the `ss` command and only launch once ports `9002`, `9003`, and `9004` are free.

### 4. Python Dependencies (PyTorch)
The `drone_controller_node` runs the neural network policy and requires PyTorch. You can run it either:
* **Natively on the host**: Install CPU-only PyTorch:
  ```bash
  pip3 install --user torch --index-url https://download.pytorch.org/whl/cpu
  ```
* **Inside Docker**: Ensure you execute the launch commands inside the Docker environment (`docker compose exec dev bash`), where the full ML/RL stack is pre-configured.

---

## 🚀 Launching the Simulation

You can launch the complete simulation loop (Gazebo server, Betaflight firmware, ROS topic bridges, state bridge, controller, and RC bridge) using a single bringup command.

```bash
source scripts/source_sitl_env.sh

ros2 launch drone_control racing_bringup.launch.py \
  sitl:=true \
  sitl_world:="$PWD/sim/gazebo/worlds/sitl_racing_custom.sdf" \
  view_stream:=false \
  publish_raw:=false \
  publish_rgb:=false \
  use_vio:=false \
  use_ekf:=false \
  controller_type:=rl  # use mpc for the MPC path
```

Avoid putting project-specific SITL exports in `~/.bashrc`. The helper above detects the installed ROS 2 distro, sources the workspace, keeps WSL/Fast DDS discovery on localhost, and applies the Gazebo GPU defaults needed by WSLg.

### Headless vs. GUI Mode
By default, the simulation starts Gazebo in **headless mode** (running only the physics server `gz-sim-server`) to minimize resource consumption and avoid graphical driver issues.

If you want to spawn the graphical interface to visually monitor the drone, open another terminal and start the Gazebo GUI:

```bash
gz sim -g
```

Only one Gazebo server should be running for the racing world before attaching the GUI. If an older server is still alive, the GUI can attach to that stale world while Betaflight talks to the fresh racing bringup server. That makes Betaflight altitude and the visible drone disagree. Check for duplicates with:

```bash
pgrep -af 'gz sim|gz-sim-main|betaflight_SITL|racing_bringup'
gz topic -i -t /world/sitl_racing_custom/stats
```

The stats topic should show one publisher. If it shows two, stop the stale Gazebo processes using the cleanup commands from the preflight section and restart racing bringup.

On WSL, Gazebo may need explicit Qt and Mesa settings to use the Windows GPU through Mesa D3D12:

```bash
GALLIUM_DRIVER=d3d12 QT_QPA_PLATFORM=xcb gz sim -g
```

---

## 🧯 Troubleshooting

### Shell Environment Pitfalls

Use `source scripts/source_sitl_env.sh` in each SITL terminal instead of relying on `~/.bashrc`. Global shell setup can make ROS behavior differ between machines, especially if `~/.bashrc` sources a ROS distro automatically or a login shell activates a Python virtual environment before ROS is sourced.

On WSL, the helper keeps local SITL discovery on localhost. This avoids Fast DDS choosing Docker or virtual switch interfaces for discovery, which can make `ros2 topic list`, `joy_tx_emulator`, and `sitl_rc_bridge` appear to be in different ROS graphs.

### Gazebo GUI Uses CPU Rendering on WSL

If Gazebo opens slowly or `glxinfo -B` reports `llvmpipe`, WSL is using Mesa's CPU renderer instead of the Windows GPU:

```text
OpenGL renderer string: llvmpipe (LLVM ...)
Accelerated: no
```

First check whether the D3D12 path works when forced:

```bash
GALLIUM_DRIVER=d3d12 glxinfo -B
```

Expected GPU-backed output contains the real adapter name and `Accelerated: yes`, for example:

```text
OpenGL renderer string: D3D12 (AMD Radeon RX 7900 XT)
Accelerated: yes
```

Use the same environment when starting Gazebo:

```bash
GALLIUM_DRIVER=d3d12 QT_QPA_PLATFORM=xcb gz sim -g
```

### Quick Simulator Reset

To reset the drone and world model state without restarting racing bringup, run:

```bash
scripts/reset_sitl_sim.sh
```

The reset uses Gazebo's model pose service to put the `drone` model back at the racing-world spawn pose (`0 0 0.5`) without restarting racing bringup or rewinding simulation time.

The TUI joystick has the same action on `X`:

```bash
ros2 run betaflight_msp_controller joy_tx_emulator
```

Pressing `X` disarms the virtual transmitter, drops throttle to minimum, pulses Joy button `0` (`SIM RESET`), and requests the Gazebo reset.

If the Gazebo GUI fails with an Ogre / GLX error like `currentGLContext was specified with no current GL context`, keep `QT_QPA_PLATFORM=xcb` set. This avoids the problematic Qt Wayland path used by some Gazebo GUI configurations under WSLg.

### Drone Falls Through the Ground Plane

This is expected in the current SITL world when the vehicle is not producing lift. The `ground_plane` in `sim/gazebo/worlds/sitl_racing.sdf` is visual-only: it has a `<visual>` plane but no `<collision>` plane. That means it looks like a floor but is not a physical contact surface.

The default `sitl_racing_custom.sdf` world includes a simple physical floor and
raised launch pad. The SITL plugin holds the disarmed model at its `z=0.5`
reset pose until any motor command exceeds `0.10`; this launch restraint avoids
contact-solver impulses contaminating controller tests. It releases once per
simulation run and does not interfere after takeoff. If you switch back to
`sitl_racing.sdf`, the ground remains visual-only.

If the drone sinks while armed, debug motor/RC flow rather than ground contact:

```bash
ss -ulpn | rg '900[234]'
ros2 topic hz /odometry
ros2 topic echo /joy --once
```

For a quick lift check, run the manual throttle takeoff verifier below. Low throttle should stay near the launch height and full throttle should increase `/odometry` `z`.

---

## 🎯 Verification: Manual Throttle Takeoff

The fastest end-to-end check is to emulate a joystick, arm Betaflight, and command full throttle while sampling `/odometry`.

Keep the bringup running in terminal 1. In terminal 2, run:

```bash
source scripts/source_sitl_env.sh

python3 - <<'PY'
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry

class SitlVerifier(Node):
    def __init__(self):
        super().__init__("sitl_manual_throttle_verifier")
        self.pub = self.create_publisher(Joy, "/joy", 10)
        self.z = None
        self.vz = None
        self.create_subscription(Odometry, "/odometry", self.odom_cb, 10)

    def odom_cb(self, msg):
        self.z = msg.pose.pose.position.z
        self.vz = msg.twist.twist.linear.z

    def send_joy(self, throttle_axis, armed):
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "sitl_verify"
        msg.axes = [0.0, 0.0, 0.0, float(throttle_axis)]
        msg.buttons = [0, 0, 0, 0, 1 if armed else 0, 0, 0, 0]
        self.pub.publish(msg)

    def phase(self, name, throttle_axis, armed, seconds):
        samples = []
        start = time.time()
        last_print = 0.0
        while time.time() - start < seconds:
            self.send_joy(throttle_axis, armed)
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.z is not None:
                samples.append(self.z)
            elapsed = time.time() - start
            if elapsed - last_print >= 1.0 and self.z is not None:
                print(f"{name}: z={self.z:.3f} vz={self.vz:.3f}", flush=True)
                last_print = elapsed
            time.sleep(0.01)
        if samples:
            print(
                f"{name}: start={samples[0]:.3f} end={samples[-1]:.3f} "
                f"min={min(samples):.3f} max={max(samples):.3f}",
                flush=True,
            )

rclpy.init()
node = SitlVerifier()

deadline = time.time() + 2.0
while node.z is None and time.time() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)

print(f"initial: z={node.z:.3f} vz={node.vz:.3f}", flush=True)
node.phase("armed_low", -1.0, True, 2.0)
node.phase("full_throttle", 1.0, True, 4.0)
node.phase("disarm", -1.0, False, 0.4)

node.destroy_node()
rclpy.shutdown()
PY
```

Expected result:

* `armed_low` stays near the virtual pad height (`z ~= 0.5`).
* `full_throttle` increases `z` clearly within a few seconds.

Known-good verification output from the repaired environment:

```text
initial: z=0.500 vz=0.006
armed_low: start=0.500 end=0.498 min=0.498 max=0.500
full_throttle: start=0.498 end=16.761 min=0.498 max=16.761
```

Stop the bringup with `Ctrl-C` after verification.

---

## 🎯 Verification: RL Interface vs. Learned Policy

`controller_type:=rl` selects the RL **controller interface** and observation /
action adapter. It does not by itself prove that a neural-network policy is
active. The qualified default is:

```text
RL_RESIDUAL_POLICY=true
RL_RESIDUAL_SCALE=0
```

At zero residual scale, checkpoint inference is skipped and the controller uses
only the deterministic position/velocity outer loop. This is the currently
qualified Boreas SITL policy. It is hand-designed and Gazebo-tested; it was not
trained by PPO.

### Validate the qualified nominal controller

Run the validator in a clean terminal with no other racing bringup or Gazebo
server using the SITL ports:

```bash
source scripts/source_sitl_env.sh
unset RL_POLICY_PATH RL_RESIDUAL_POLICY RL_RESIDUAL_SCALE \
  RL_RESIDUAL_MASK RL_POLICY_OUTPUT_TANH

python3 scripts/simulation/validate_sitl_controller.py rl \
  --arm --sample-seconds 20 \
  --goal 1 0 0.5 \
  --max-final-goal-error 0.15
```

`--goal X Y Z` is published on `/drone_goal` after the controller is ready. It
therefore overrides launch defaults even when
`--goal-delay-after-arm` is zero. A verified `X=1 m` run finished at
approximately `(0.987, 0.015, 0.478) m`, with `0.029 m` final 3-D error.

For a step-response test, keep the initial hover goal until a chosen delay:

```bash
python3 scripts/simulation/validate_sitl_controller.py rl \
  --arm --sample-seconds 10 \
  --goal 1 0 0.5 --goal-delay-after-arm 3 \
  --max-final-goal-error 0.15 --min-goal-error-reduction 0.8
```

The validator rejects stale SITL processes during preflight and also verifies
that its newly launched bringup remains alive. Stop old sessions before retrying:

```bash
pgrep -af 'racing_bringup|betaflight_SITL|gz sim|sitl_rc_bridge'
```

### Run a learned residual checkpoint explicitly

A learned checkpoint is evaluated only when all required environment variables
are set and the residual scale is nonzero:

```bash
export RL_POLICY_PATH=/absolute/path/to/agent_72000.pt
export RL_RESIDUAL_POLICY=true
export RL_RESIDUAL_SCALE=0.15
export RL_RESIDUAL_MASK=0,1,1,0
export RL_TRAINING_MAX_THRUST_N=35.9315656
export RL_POLICY_OUTPUT_TANH=true

python3 scripts/simulation/validate_sitl_controller.py rl \
  --arm --sample-seconds 20 \
  --goal 1 0 0.5 \
  --max-final-goal-error 0.15
```

Do not interpret a finite command stream as a policy pass. The result must show
`betaflight_motors_active: true`, `goal_tracking_passed: true`, and
`passed: true`. No learned residual is currently promoted: the v8--v12
candidates regressed the nominal controller in paired Isaac or armed Gazebo
tests. Leave `RL_RESIDUAL_SCALE=0` for the qualified default.

### Publish a goal during an interactive bringup

When running `racing_bringup.launch.py` directly, publish the world-frame goal:

```bash
ros2 topic pub /drone_goal geometry_msgs/msg/Point \
  "{x: 1.0, y: 0.0, z: 0.5}" --rate 2
```

The goal is sticky in `drone_controller_node`, but a low-rate publisher makes
startup ordering explicit. Stop it with `Ctrl-C` after the controller reports
the accepted goal.

### Verify active telemetry

In a separate terminal, monitor the key control-loop topics (~100 Hz):

```bash
ros2 topic hz /drone_state_sim
ros2 topic hz /drone_observation
ros2 topic hz /drone_attitude_cmd
```

You can also check the Gazebo sensor ingress topics:

```bash
ros2 topic hz /imu0
ros2 topic hz /odometry
```

---

## 🎮 Manual Control / TUI Control (Emulating /joy)

To manually fly, arm, or override the drone during SITL simulation, you can run the terminal-based Joy Emulator. It publishes standard `sensor_msgs/msg/Joy` messages to the `/joy` topic, which the `sitl_rc_bridge` node listens to for arming and override inputs.

To launch the emulator in a new terminal:

```bash
source scripts/source_sitl_env.sh
ros2 run betaflight_msp_controller joy_tx_emulator
```

### Controls Layout
* **Pitch / Roll**: `W`/`S` (Pitch) & `A`/`D` (Roll)
* **Throttle / Yaw**: `Up`/`Down` arrows (Throttle) & `Left`/`Right` arrows (Yaw)
* **AUX / Modes**:
  * `Space`: Toggle ARM/DISARM (AUX 1 / Button 4)
  * `2`: Toggle Override control (AUX 2 / Button 5)
  * `3` / `4`: Toggle AUX 3 / AUX 4 (Buttons 6 & 7)
* **Utilities**:
  * `C`: Center all attitude/yaw sticks (Roll, Pitch, Yaw)
  * `R`: Reset throttle stick to minimum (`-1.0`)
  * `Q`: Quit the TUI emulator safely

---

## ⚙️ Configuration & Customization

* **Gazebo World File**: Defined in `sim/gazebo/worlds/sitl_racing.sdf`. Modify this file to add obstacles, gates, or custom lighting.
* **Boreas Physical Profile**: Defined in `config/airframes/boreas_4s.yaml`.
  Never tune the SDF and MPC independently.
* **Betaflight Configuration**: CLI parameters for the SITL flight controller are located in `docs/docs/files/betaflight_conf/sitl_config.txt`.

### Custom Gate Placement

Reusable gates should live as Gazebo models under `sim/gazebo/models/`. The current ETHRC gate model is:

```text
sim/gazebo/models/race_gate_ethrc/
├── model.config
├── model.sdf
├── meshes/
│   ├── banner_yz.obj
│   ├── banner_yz_horizontal.obj
│   └── banner_yz_vertical.obj
└── materials/
    └── textures/
        ├── bottom.jpg
        ├── left.jpg
        ├── right.jpg
        └── top.jpg
```

`model.sdf` defines the gate geometry, collisions, UV-mapped banner meshes, and texture paths. Keep the collision geometry simple, usually boxes, and use meshes/textures only for visuals. This keeps Gazebo contact behavior stable while still allowing branded gate artwork.

Place gates in a world file with `<include>` blocks. For example, in `sim/gazebo/worlds/sitl_racing_custom.sdf`:

```xml
<include>
  <uri>model://race_gate_ethrc</uri>
  <name>gate_1</name>
  <pose>5 0 1.0 0 0 0</pose>
</include>

<include>
  <uri>model://race_gate_ethrc</uri>
  <name>gate_2</name>
  <pose>10 4 1.0 0 0 0.65</pose>
</include>
```

Gazebo poses are:

```text
x y z roll pitch yaw
```

For gates:

* `x` / `y`: horizontal track position in meters.
* `z`: gate center height in meters. For `race_gate_ethrc`, `z=1.0` puts the 1 m by 1 m opening roughly between `0.5 m` and `1.5 m`.
* `yaw`: rotation around the vertical axis in radians. `1.57` is about `90 deg`.
* `roll` and `pitch`: usually keep these at `0` unless you intentionally want tilted gates.

Before launching SITL, validate the model and world:

```bash
gz sdf -k sim/gazebo/models/race_gate_ethrc/model.sdf
SDF_PATH="$PWD/sim/gazebo/models:${SDF_PATH-}" \
  gz sdf -k sim/gazebo/worlds/sitl_racing_custom.sdf
```

For direct Gazebo testing:

```bash
export GZ_SIM_RESOURCE_PATH="$PWD/sim/gazebo/models:${GZ_SIM_RESOURCE_PATH}"
GALLIUM_DRIVER=d3d12 QT_QPA_PLATFORM=xcb \
  gz sim sim/gazebo/worlds/sitl_racing_custom.sdf
```

For full SITL, pass the world through `sitl_world`:

```bash
ros2 launch drone_control racing_bringup.launch.py \
  sitl:=true \
  sitl_world:="$PWD/sim/gazebo/worlds/sitl_racing_custom.sdf"
```

---

## 🧩 Repair Notes and Findings

These are the important implementation details behind the currently verified SITL path.

### EthrcBetaflightSITLPlugin (`src/sitl_gazebo_plugins`)

* The Gazebo plugin must load from `install/sitl_gazebo_plugins/lib`; `scripts/source_sitl_env.sh` and `racing_bringup.launch.py` add this path to `GZ_SIM_SYSTEM_PLUGIN_PATH`.
* The FDM packet ABI sent to Betaflight must match the SITL firmware. The packet includes `timestamp`, IMU angular velocity, IMU linear acceleration, quaternion, velocity, position, and `pressure`.
* The SDF `LiftDrag` rotor path was not sufficient for reliable takeoff in this model. The working path applies direct world force from decoded normalized motor outputs.
* Measured maximum static thrust is `8.9828914 N` (916 gf) per propeller, or
  `35.9315656 N` total. With the `0.8351086 kg` URDF this is a `4.387:1`
  thrust-to-weight ratio. The intermediate curve remains quadratic and
  provisional pending a multi-point sweep.
* The plugin publishes motor command, filtered RPM, per-motor thrust, applied
  wrench, and calibration-status diagnostics under `/drone/sim/`.
* Betaflight QUADX motor order is rear-right, front-right, rear-left,
  front-left. The Boreas rotor bindings follow that order explicitly.
* The virtual IMU is mounted with a 180-degree roll so Betaflight receives FRD
  measurements. The ROS state bridge converts angular velocity back to FLU.
* Before launch, motor commands below `0.10` hold the model at the reset pose
  and zero its velocity. This prevents the physical launch pad from injecting
  a large contact impulse.

### Gazebo SDF

* The base collision in `sim/gazebo/models/drone_sitl/model.sdf` is a simple box. Mesh collision and rotor collisions caused unstable contact behavior.
* The `sitl_racing.sdf` ground plane is visual-only. The previous physical ground collision triggered DART/ODE AABB aborts during SITL.

### Betaflight SITL (`ext/anemos-betaflight`)

* UDP RX is treated as a held latest-value stream after the first valid packet. This prevents FC-time RXLOSS between wall-time UDP packets.
* UDP RC packets are copied into a coherent flight-control snapshot before
  Betaflight reads the channels. This prevents a receiver-thread update from
  mixing throttle or AUX values from different packets in one FC cycle.
* `RX_PROVIDER_UDP` starts with the full supported channel count so the first UDP RC packet is consumed.
* Motor output packets are sent continuously from `pwmCompleteMotorUpdate()`; they are not gated by the old FDM update mutex.
* PID numerators, disabled feedforward, and the linear 500 deg/s ACTUAL rate
  profile mirror `boreas_jijieh743.txt`. Physical motor pin reordering remains
  hardware-only; SITL retains logical QUADX order.

### `sitl_rc_bridge`

* The UDP RC heartbeat runs at `500 Hz`. Betaflight SITL can advance FC time faster than wall time, so `50 Hz` can be too slow.
* Manual `/joy` axes map directly to Roll, Pitch, Throttle, and Yaw when override is disabled.
* AUX2 is held low in SITL so Betaflight does not enter `BOXMSPOVERRIDE` and ignore the UDP RX stream.
* Betaflight pitch input follows the RC-stick convention: positive means
  nose-down. Controller adapters must not treat this channel as mathematical
  positive FRD pitch. Roll is unchanged; FLU yaw is inverted for Betaflight's
  clockwise-positive yaw input.

### Debugging Checklist

If the drone arms but does not climb:

1. Confirm Betaflight sees RC input:
   ```text
   rcData ... T:2000 ... A1:2000 ... boxarm:1 armed:1
   ```
2. Confirm Gazebo owns motor UDP port `9002`:
   ```bash
   ss -ulpn | rg '900[234]'
   ```
3. Confirm `/odometry` is streaming:
   ```bash
   ros2 topic hz /odometry
   ```
4. Rebuild the three moving pieces:
   ```bash
   ext/aeroloop_gazebo/build_plugin.sh
   (cd ext/anemos-betaflight && make TARGET=SITL -j$(nproc))
   colcon build --symlink-install --packages-up-to drone_control betaflight_msp_controller gazebo_ros_bridges
   ```
