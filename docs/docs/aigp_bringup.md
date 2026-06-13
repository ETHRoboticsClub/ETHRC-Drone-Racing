# AIGP Simulation Guide

**Page Owner:** [Members of the Aeronautics Division](https://github.com/orgs/ETHRoboticsClub/teams/aeronautics/)

---

The **AI Grand Prix (AIGP)** simulator is supported as a high-fidelity virtual qualification environment for autonomous racing. Since AIGP does not support standard PX4/MAVSDK connections directly, we utilize a dedicated MAVLink bridge node and camera driver.

---

## 🏗️ Architecture & Communication

The simulator runs on a host machine (e.g. Windows) and transmits telemetry/images over UDP sockets:

```
[ AIGP Simulator ]
   │
   ├── MAVLink (UDP:14550) ──▶ [ aigp_bridge_node ] ──▶ /drone_state_sim (13-element state)
   │                                                 ──▶ /imu0 (sensor_msgs/Imu)
   │
   └── JPEG chunks (UDP:5600) ──▶ [ VisionDriverNode ] ──▶ /cam0/image_raw (mono8)
                                                       ──▶ /cam0/image_rgb (rgb8)
```

---

## 🛠️ Components

### 1. `aigp_bridge_node`
A dedicated Python ROS2 node that replaces the MAVSDK client. It handles:
- **Telemetry Parsing:** Reads `ODOMETRY` and `ATTITUDE` packets, and publishes `/drone_state_sim` (Float64MultiArray) to feed the controller.
- **Clock Synchronization:** Sends 10Hz `TIMESYNC` requests to keep simulator and ROS clocks aligned.
- **Control Translation:** Converts `/drone_attitude_cmd` (deg/s) into MAVLink `set_attitude_target_send` body rates (rad/s).
- **Direct IMU Stream:** Optionally republishes `HIGHRES_IMU` directly as `/imu0` (`sensor_msgs/msg/Imu`) to prevent port conflicts with C++ adapters.

### 2. `VisionDriverNode`
A C++ component inside `aigp_camera_driver` that listens on port `5600`, reassembles chunked-JPEG UDP packets, decodes them using OpenCV, and publishes standard ROS image topics.

---

## 🚀 Running the Simulation

### Step 1: Run the Simulator
Launch `FlightSim.exe` on the host machine. It will start sending UDP streams to `127.0.0.1`.

### Step 2: Build & Start Container
By default, the container starts with CPU/software rendering (which runs on all systems). Build and run the `aigp` compose service:

```bash
# Build the container
docker compose build dev

# Start the container
docker compose up aigp -d

# Exec into the container
docker compose exec aigp bash
```

> [!TIP]
> **Enabling Host GPU Acceleration:**
> If you have a dedicated GPU and want to enable hardware-accelerated OpenGL/decoding inside the container, configure it in `docker-compose.yaml` (under the `aigp` service):
> 
> * **For Intel/AMD GPUs:** Mount the host direct rendering interface:
>   ```yaml
>   devices:
>     - /dev/dri:/dev/dri
>   ```
> * **For NVIDIA GPUs:** Add the NVIDIA Container Toolkit runtime config:
>   ```yaml
>   environment:
>     - NVIDIA_VISIBLE_DEVICES=all
>     - NVIDIA_DRIVER_CAPABILITIES=all
>   deploy:
>     resources:
>       reservations:
>         devices:
>           - driver: nvidia
>             count: all
>             capabilities: [gpu]
>   ```

### Step 3: Launch AIGP Bringup
Within your ROS2 workspace, build and run the launchfile:

```bash
colcon build --packages-select betaflight_mavlink_bridge drone_control aigp_camera_driver aigp_imu_driver
source install/setup.bash

# Run bringup
ros2 launch drone_control aigp_bringup.launch.py controller_type:=rl
```

---

## 🎮 Arming & Controlling the Simulator

Unlike physical Betaflight deployments using MSP, the AIGP simulator uses MAVLink commands for control:

### 1. Manual / Joystick Control
If you have a joystick/gamepad connected:
- **R1 Button:** Arm / Disarm the drone.
- **L1 Button:** Toggle autonomous mode.
- **A / Cross Button:** Reset the simulation level.
- **Joystick axes:** Controls roll rate, pitch rate, yaw rate, and thrust manually when autonomous mode is disabled.

### 2. ROS2 Service Calls
You can also control simulator state using standard service calls:
- **Arming:**
  ```bash
  ros2 service call /arm_drone std_srvs/srv/SetBool "{data: true}"
  ```
- **Resetting the simulation:**
  ```bash
  ros2 service call /reset_sim std_srvs/srv/Trigger
  ```

> [!NOTE]
> **Headless / Joystick-Free Autonomous Mode:**
> By default, `is_auto` is set to `True` in `aigp_bridge_node`. This allows you to perform autonomous runs immediately after arming the drone via the service call, without needing a physical gamepad or joystick connected.

---

## 🔍 Troubleshooting & WSL2 Network Tuning

### 1. ROS2 Daemon Hanging (`ros2 topic list` or `ros2 service call` hangs)
In WSL2 mirrored networking mode, the ROS 2 daemon can occasionally become unresponsive or time out. 
* **Bypass the Daemon:** Set the environment variable `ROS_NO_DAEMON=1` or append `--no-daemon` to `ros2 topic` commands:
  ```bash
  ros2 topic list --no-daemon
  ros2 topic hz /drone_state_sim --no-daemon
  ```
* **Local Loopback Configuration:** If nodes cannot find each other, ensure CycloneDDS is forced to loopback (`lo`) by setting `CYCLONEDDS_URI` to point to `src/cyclonedds.xml`. This configuration is already set up in the `docker-compose.yaml` file.
