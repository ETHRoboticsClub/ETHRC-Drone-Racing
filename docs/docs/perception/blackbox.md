# Betaflight Blackbox Serial Driver

**Page Owner:** [Ilyas Seckin](https://github.com/ilysec)

The `betaflight_blackbox_driver` is a high-performance ROS 2 node designed to stream high-rate IMU and RPM telemetry from a Betaflight flight controller to the NVIDIA Jetson Orin NX.

## Overview

Unlike standard MSP-based telemetry, this driver utilizes the **Betaflight Blackbox** serial protocol. This allows for significantly higher data rates (up to 2kHz+), which is critical for low-latency control and high-speed visual-inertial odometry (VIO).

### Key Features
- **High-Rate IMU**: Streams Accelerometer and Gyroscope data downsampled to the requested 100-200 Hz range.
- **RPM Feedback**: Parses real-time motor RPM values from the Blackbox stream (requires Bidirectional DShot).
- **CRC-8 Protection**: Uses a custom frame-level CRC-8 (DVB-S2) checksum to verify serial stream integrity, completely eliminating UART-induced spikes or bit-flip noise.
- **Hardware Timestamping**: Integrates with the Jetson Hardware Timestamping Engine (HTE) for microsecond-accurate alignment of IMU data.
- **Low Latency**: Uses Linux serial low-latency mode and real-time thread priority.

---

## ROS 2 Interface

### Parameters

| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `port` | `string` | `/dev/ttyTHS1` | Serial port connected to the Betaflight UART. |
| `baud` | `int` | `1500000` | Baud rate for communication (1.5Mbps recommended). |
| `imu_topic` | `string` | `/imu0` | Topic to publish `sensor_msgs/Imu` data. |
| `rpm_topic` | `string` | `/motor_rpm` | Topic to publish `std_msgs/Float64MultiArray` RPM data. |
| `battery_topic` | `string` | `/battery` | Topic to publish `sensor_msgs/BatteryState` data. |
| `target_hz` | `int` | `150` | Target frequency for the published IMU messages. Values outside 100-200 Hz are clamped. |
| `motor_poles` | `int` | `14` | Number of magnetic poles in the motors (used for RPM calculation). |
| `vbat_scale` | `double` | `0.01` | Scaling factor for battery voltage (Betaflight defaults to 0.01V/unit). |
| `use_blackbox_time_for_stamp` | `bool` | `false` | If true, derive IMU header stamps from Betaflight blackbox sample time. By default the paced ROS publish timer is used for strictly monotonic wall-clock stamps. |
| `validate_imu_stream` | `bool` | `true` | Reject decoded IMU frames that fail timestamp, range, or jump sanity checks before publishing. |
| `reject_zero_imu` | `bool` | `true` | Reject frames where all gyro and accel axes decode to exactly zero. |
| `max_frame_delta_us` | `int` | `10000` | Maximum accepted Betaflight P-frame time delta. Larger values are treated as parser/serial corruption. |
| `max_gyro_raw_deg_s` | `double` | `4000.0` | Maximum absolute decoded gyro value in deg/s before a frame is rejected. |
| `max_accel_g` | `double` | `32.0` | Maximum absolute decoded acceleration in g before a frame is rejected. |
| `max_gyro_jump_deg_s` | `double` | `2500.0` | Maximum per-frame gyro jump accepted for P-frames. |
| `max_accel_jump_g` | `double` | `16.0` | Maximum per-frame accel jump accepted for P-frames. |
| `debug_raw` | `bool` | `false` | If true, dumps raw binary data to `/tmp/capture.bbl`. |
| `enable_msp_control` | `bool` | `false` | Enables shared UART control for MSP commands (experimental). |

### Published Topics
- **`/imu0`** (`sensor_msgs/msg/Imu`): High-rate fused accelerometer and gyroscope data. Published from the latest decoded sample on a paced ROS timer with best-effort sensor-data QoS and queue depth 1.
- **`/motor_rpm`** (`std_msgs/msg/Float64MultiArray`): Array of 4 values representing the mechanical RPM of each motor.
- **`/battery`** (`sensor_msgs/msg/BatteryState`): Real-time battery voltage telemetry.

---

## Hardware Setup

### Wiring
Connect the Betaflight UART TX pad to the Jetson RX pin. Ensure a common ground.
Refer to the [Betaflight Setup](../betaflight.md) guide for specific pinouts.

### Jetson Clock Configuration
To support 1.5Mbps, the Jetson UART clock must be boosted to 80MHz:
```bash
echo 80000000 | sudo tee /sys/kernel/debug/bpmp/debug/clk/uarti/rate
```

---

## Betaflight Configuration

To enable the data required for this driver, use the Betaflight CLI:

### 1. Enable Blackbox Serial
```bash
feature BLACKBOX
set blackbox_device = SERIAL
set blackbox_mode = ALWAYS
serial 0 128 1500000 57600 115200 1500000
```

### 2. Enable RPM Telemetry (Optional)
To receive RPM feedback, you must enable **Bidirectional DShot** in the Motors tab and ensure the following CLI settings:
```bash
set blackbox_disable_motors = OFF
set blackbox_disable_rpm = OFF
```

### 3. Minimize Bandwidth
To ensure stable 2kHz operation, disable unnecessary fields. Note that **Battery** fields should be kept `OFF` (enabled) to receive voltage data:
```bash
set blackbox_disable_pids = ON
set blackbox_disable_rc = ON
set blackbox_disable_setpoint = ON
set blackbox_disable_bat = OFF # Enable for Battery Voltage
set blackbox_disable_alt = ON
set blackbox_disable_rssi = ON
set blackbox_disable_debug = ON
set blackbox_disable_attitude = ON
```

---

## Troubleshooting

### eRPM values are all zero
If the node logs `eRPM values are all zero but motor outputs are non-zero`, it means the Blackbox is sending RPM fields but they contain no data.
- **Solution**: Ensure **Bidirectional DShot** is properly configured and working in the Betaflight Configurator "Motors" tab.

### IMU jumps to zero or very large values
The driver validates every decoded IMU frame before it is published. If a bad P-frame is detected, it is discarded and the parser waits for the next I-frame so corrupted deltas do not poison later samples.

- Watch the `Blackbox stream health` log line. A clean stream should show accepted frames with zero or very rare rejected frames and no dropped serial bytes.
- If rejections happen in bursts, enable `debug_raw:=true` and inspect `/tmp/capture.bbl` so the raw UART stream can be compared against decoded output.
- If the vehicle can genuinely exceed the defaults, loosen `max_gyro_raw_deg_s`, `max_accel_g`, `max_gyro_jump_deg_s`, or `max_accel_jump_g`. Do this only after checking that the rejection reason matches real vehicle motion.

### No data received
- **Check Wiring**: Swap RX/TX wires if no bytes are detected.
- **Check Clock**: Ensure the Jetson UART clock is set to 80MHz.
- **Check Permissions**: Ensure your user is in the `dialout` group.
