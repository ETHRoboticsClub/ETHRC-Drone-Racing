# AIGP simulator drivers — workflow

**Owner:** [Jules Schmutz](https://github.com/juliomathis)

## Network setup (Windows host + WSL2 + Docker)

`FlightSim.exe` runs on Windows and sends to `127.0.0.1`: MAVLink to UDP 14550, vision to UDP 5600. For the ROS nodes in WSL2 to receive them:

1. Mirrored networking in `C:\Users\<you>\.wslconfig`, then `wsl --shutdown`:
   ```ini
   [wsl2]
   networkingMode=mirrored
   kernelCommandLine = clocksource=hyperv_clocksource_tsc_page
   ```
2. Run the container with `network_mode: host` (the repository's `docker-compose.yaml` does).
3. Raise the socket buffers on the WSL host so the colour stream is not dropped:
   ```bash
   sudo sysctl -w net.core.rmem_max=2147483647 net.core.wmem_max=2147483647
   printf 'net.core.rmem_max=2147483647\nnet.core.wmem_max=2147483647\n' | sudo tee /etc/sysctl.d/60-cyclonedds.conf
   sudo sysctl --system
   ```

## Build

```bash
colcon build --symlink-install --packages-select aigp_camera_driver aigp_imu_driver
source install/setup.bash
```

`aigp_imu_driver` needs the MAVLink C headers (`mavlink/v2.0/common/mavlink.h`); `aigp_camera_driver` needs OpenCV.

## Run

Full simulator stack (camera driver + bridge IMU + estimator + policy):

```bash
ros2 launch drone_control aigp_bringup.launch.py
```

Drivers alone:

```bash
ros2 launch aigp_camera_driver aigp_camera.launch.py            # config:=<yaml> to override
ros2 launch aigp_imu_driver aigp_imu.launch.py                  # not together with aigp_bringup
```

For VIO in the simulator, align the clocks first:

```bash
ros2 launch aigp_imu_driver aigp_imu.launch.py config:=/path/to/imu_params_receipt.yaml   # use_sim_timestamp: false
```

## Verify

```bash
ros2 topic hz /cam0/image_rgb                                       # simulator frame rate
ros2 topic echo --once --qos-reliability best_effort /cam0/camera_info
ros2 topic hz /imu0                                                  # ~100+ Hz
ros2 topic echo --once --qos-reliability best_effort /imu0
ros2 run rqt_image_view rqt_image_view                               # /cam0/image_rgb, Best Effort
```

`ros2 topic hz` works on best-effort topics; `echo` needs `--qos-reliability best_effort`.

## Troubleshooting

| Symptom | Cause | Fix |
| :--- | :--- | :--- |
| `failed to bind UDP port` | Port already taken (another driver, the bridge, or a stale process) | `ss -ulnp` to find who holds 5600 or 14550; stop the other process |
| No images, `hz` prints nothing | Packets not reaching WSL, or no subscriber (decoding is lazy) | Check mirrored networking; subscribe to the topic |
| Images stop after restarting the simulator | Frame ids reset | Restart the camera driver |
| `Dropped packet: payload truncated` or `JPEG decode failed` in bursts | UDP loss | Raise `rmem_max`; reduce simulator resolution/rate |
| `/imu0` at 0 Hz with the bridge running | Both the bridge and `aigp_imu_driver` bound 14550 | Run only one |
| OpenVINS never initializes in the simulator | Camera and IMU stamps on different clocks | `use_sim_timestamp:=false` on the IMU driver |
