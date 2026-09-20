# AIGP IMU Driver

**Page Owner:** [Jules](https://github.com/juliomathis)

## Overview

Adapter node for the **AI Grand Prix Virtual Qualifier** simulator MAVLink
interface. It reads the simulator's `HIGHRES_IMU`
telemetry and republishes it as `sensor_msgs/Imu` on `/imu0`

```
AIGP simulator -> MAVLink (HIGHRES_IMU) -> imu_bridge_node -> /imu0 (sensor_msgs/Imu)
```

A C++ `rclcpp` component (composable, intra-process zero-copy). MAVLink is parsed
directly with the standard mavlink C library — the simulator is not a
PX4/ArduPilot autopilot (HEARTBEAT reports `autopilot=GENERIC`, component 200),
so MAVSDK never attaches.

## Usage

```bash
ros2 launch aigp_imu_driver aigp_imu.launch.py
# override the parameter file (or just the connection):
ros2 launch aigp_imu_driver aigp_imu.launch.py config:=/path/to/params.yaml
```

### Connection string

`connection_url` defaults to **`udpin://0.0.0.0:14550`** — the port the AI-GP
simulator pushes its MAVLink stream to (`PyAIPilotExample/main.py`:
`SIM_SERVER_UDP_PORT = 14550`).

## Verifying

```bash
ros2 launch aigp_imu_driver aigp_imu.launch.py
ros2 topic hz /imu0                                          # ~100+ Hz against the sim
ros2 topic echo --once --qos-reliability best_effort /imu0   # see one sample
```

`/imu0` uses **best-effort** (sensor) QoS, so a default *reliable* `ros2 topic
echo` shows nothing — pass `--qos-reliability best_effort`.


## Running against the simulator from Windows + WSL2 + Docker

`FlightSim.exe` runs natively on Windows and pushes its UDP streams to
`127.0.0.1` (MAVLink → 14550, vision → 5600). To reach the nodes running in WSL2:

1. **WSL2 mirrored networking** — in `C:\Users\<you>\.wslconfig`:
   ```ini
   [wsl2]
   networkingMode=mirrored
   kernelCommandLine = clocksource=hyperv_clocksource_tsc_page
   ```