# Betaflight Setup for High-Rate IMU

This document describes the configuration required to enable 2000Hz IMU streaming from Betaflight to NVIDIA Jetson Orin NX.

## Hardware Wiring

| Jetson Pin (A603) | Signal Name | Betaflight Pad | Note |
|---|---|---|---|
| Pin 10 | UART1_RX | TXn | Blackbox Data Stream (Requires RX/TX swap check) |
| Pin 16 | GPIO3_PBB.01 | LED_STRIP | HTE Sync Trigger |
| Pin 6 | GND | GND | Common Reference |

> [!CAUTION]
> If you receive 0 bytes on the serial port, verified by `xxd` or `record_imu.py`, **swap the physical RX and TX wires**. UART labeling is often inconsistent between boards.

## Jetson Configuration

1. **Enable HTE**: Ensure the Hardware Timestamping Engine is enabled in the device tree for microsecond alignment.
2. **UART Clock (80MHz)**: The standard 48MHz clock is insufficient for 1.5Mbps. You MUST set the BPMP clock to 80MHz:
   ```bash
   echo 80000000 | sudo tee /sys/kernel/debug/bpmp/debug/clk/uarti/rate
   ```
   > [!IMPORTANT]
   > This setting is volatile. It MUST be reapplied after every Jetson reboot.

3. **Power Mode**: Run the Jetson in `MAXN` mode for stable high-rate serial processing:
   ```bash
   sudo nvpmodel -m 0
   sudo jetson_clocks
   ```

## Betaflight CLI Configuration (4.5.x)

For Betaflight 4.5.x, use the following commands to ensure the IMU stream matches the driver's parser:

```bash
feature BLACKBOX
set blackbox_device = SERIAL
set blackbox_sample_rate = 1/1  # Renamed from blackbox_rate_denom in 4.5
set blackbox_mode = ALWAYS

# Configure UART1 (Serial 0) for 1.5Mbps
# '128' = Blackbox; '1500000' = Baud
serial 0 128 115200 57600 115200 1500000

save
```

## Common Pitfalls & Troubleshooting

### 1. Serial Port Silence (0 bytes)
- **Problem**: `cat /dev/ttyTHS1` returns nothing.
- **Fix**: Verify `serial` and `blackbox` settings in the CLI. Always run `save`. Check if the drone is Armed (some versions only stream when armed).

### 2. Synchronization Garbage (`00 10 05` sequence)
- **Problem**: `xxd` shows repeating `00 10 05` or similar.
- **Fix**: Re-run the `echo 80000000 > ...` command.

### 3. Missing Accelerometer Data
- **Problem**: Gyro updates, but Accel is zero.
- **Fix**: Ensure `blackbox_sample_rate = 1/1`. Ensure the driver's `is_intra` check (`0x49`) is not too restrictive on frame size.

### 4. UART Buffer Overrun
- **Problem**: High CPU usage or dropped frames.
- **Fix**: Enforce `jetson_clocks`.

## WiFi Connectivity

The Jetson uses an RTL8812BU USB WiFi adapter. 

### Connecting to WiFi
Use `nmcli` to manage connections:
```bash
# Scan for networks
nmcli device wifi list

# Connect to ETHRC-Aero
sudo nmcli device wifi connect ETHRC-Aero password strongpass
```

### Troubleshooting
If the WiFi is connected but the internet is unreachable:
1. **Check Routing**: Ensure the default route via the WiFi gateway (`192.168.0.1`) has a lower metric than the USB bridge (`192.168.55.100`).
2. **Re-connect**: Sometimes a stale connection needs to be re-activated:
   ```bash
   sudo nmcli connection up ETHRC-Aero
   ```
3. **Verify IP**: Check for a valid IP in the `192.168.0.x` range on the `wlx...` interface.

## Diagnostic Scripts

A set of Python scripts is provided in `scripts/perception/betaflight/` for low-level verification:

- **`record_imu.py`**: Records raw binary data from `/dev/ttyTHS1`.
- **`parse_blackbox.py`**: Counts 'I' and 'P' frames in a recorded binary file.
- **`see_data.py`**: Decodes and prints active IMU samples from 'I' frames.

### Usage Example:
```bash
# On Jetson
python3 record_imu.py blackbox.bin
python3 parse_blackbox.py blackbox.bin
python3 see_data.py blackbox.bin
```

Remote tools in `scripts/tools/remote/`:
- **`ssh_exec.py`**: Run commands on the Jetson via the USB-C bridge (`192.168.55.1`).
- **`deploy_jetson.py`**: Robustly transfer files to the Jetson using Base64 chunking.
