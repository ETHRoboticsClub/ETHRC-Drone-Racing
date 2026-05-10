# Groundstation Web Server

**Page Owner:** [Julian Hoffman](https://github.com/JulesHere)

The groundstation web server is a ROS 2 node that subscribes to all drone
telemetry topics and exposes a live dashboard at `http://localhost:8080`.

## What It Shows

| Card | ROS Topic | Message Type |
|---|---|---|
| Battery | `/battery` | `sensor_msgs/BatteryState` |
| Throttle | `/drone/throttle` | `std_msgs/Float64` |
| IMU — Orientation / Angular Velocity / Linear Acceleration | `/imu0` | `sensor_msgs/Imu` |
| Attitude Command | `/drone_attitude_cmd` | `std_msgs/Float64MultiArray` |
| Current Target | `/drone_current_target` | `geometry_msgs/PoseStamped` |
| Position Command | `/drone_position_cmd` | `std_msgs/Float64MultiArray` |
| Motor RPM | `/motor_rpm` | `std_msgs/Float64MultiArray` |
| Camera | `/cam0/camera_info`, `/image_raw` | `sensor_msgs/CameraInfo`, `sensor_msgs/Image` |

Telemetry is pushed over a WebSocket at 30 Hz. The dashboard auto-reconnects
if the connection drops.

## Prerequisites

Python dependencies are declared in `setup.py` and installed automatically by
colcon:

```
fastapi
uvicorn
```

If you are running outside Docker and they are missing, install them manually:

```bash
pip install fastapi uvicorn
```

## Build

Run these commands from the workspace root (`/ws` inside Docker, or the repo
root on a native install):

```bash
# 1. Build the package
colcon build --packages-select groundstation

# 2. Source the install space (required in every new shell)
source install/setup.bash
```

> [!NOTE]
> If you edit only Python files you can skip the rebuild and re-source — changes
> are picked up immediately because the install space symlinks back to the source
> with `--symlink-install`. For `setup.py` changes (entry points, `data_files`)
> a full rebuild is always needed.

## Start the Server

### Convenience script

The helper script builds the package, sources the install overlay, and starts
the server in one step:

```bash
./scripts/run_groundstation.sh
```

> [!NOTE]
> The script uses `--symlink-install`, so edits to `index.html` and other
> static files are reflected immediately without a rebuild.

### Manual start

```bash
ros2 run groundstation web_server
```

Open `http://localhost:8080` in a browser. The status indicator in the
top-right turns white once the WebSocket connects.

### Optional Parameters

The host and port are ROS 2 parameters and can be overridden at launch:

```bash
ros2 run groundstation web_server --ros-args -p port:=9090
ros2 run groundstation web_server --ros-args -p host:=0.0.0.0 -p port:=8080
```

Setting `host` to `0.0.0.0` makes the dashboard reachable from other machines
on the same network.

## Troubleshooting

**`No executable found`**

The install space is out of date. Run the build and source steps above, then
retry.

**`Address already in use` on port 8080**

A previous server process did not shut down cleanly. Kill it and restart:

```bash
kill $(lsof -ti:8080)
ros2 run groundstation web_server
```

**Dashboard shows `DISCONNECTED`**

The WebSocket path is `/ws` on the same host and port as the HTTP server. Check
that no proxy or firewall is blocking WebSocket upgrades on that port.
