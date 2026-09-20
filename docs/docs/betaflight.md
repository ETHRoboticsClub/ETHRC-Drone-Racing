# Betaflight Setup for High-Rate IMU

**Page Owner:** [Dimetri Chau](https://github.com/deonixlive)

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
   > This setting is volatile. It MUST be reapplied after every Jetson reboot. A persistent systemd service `jetson-uart-init.service` is used to automate this.

3. **Power Mode**: Run the Jetson in `MAXN` mode for stable high-rate serial processing:
   ```bash
   sudo nvpmodel -m 0
   sudo jetson_clocks
   ```

4. **Persistent UART Initialization Service**:
   To automate the volatile BPMP clock adjustment on boot, configure `/etc/systemd/system/jetson-uart-init.service` with:
   ```ini
   [Unit]
   Description=Initialize Jetson UART Clock and Performance for Betaflight
   After=multi-user.target

   [Service]
   Type=oneshot
   ExecStart=/home/aero/proj/ETHRC-Drone-Racing/scripts/perception/betaflight/setup_jetson_uart.sh --apply
   RemainAfterExit=yes

   [Install]
   WantedBy=multi-user.target
   ```
   > [!IMPORTANT]
   > - The script **MUST** have executable permissions on the Jetson: `chmod +x scripts/perception/betaflight/setup_jetson_uart.sh`.
   > - Running `systemctl start jetson-uart-init.service` might print a warning: `Cannot set serial info: Invalid argument` from `setserial`. This is **harmless** and expected on Jetson platforms, as the `ttyTHS` driver does not support standard low-latency serial ioctl commands. The BPMP clock rate will still apply correctly.

5. **Enabling Tegra Combined UART (ttyTCU0) for MSP Control**:
   By default, `/dev/ttyTCU0` is claimed by the Linux kernel as a serial debug console. To use it for standard serial communication (e.g. the MSP RC simulation bridge), you must free it:
   * **Disable the Kernel Console**: Edit `/boot/extlinux/extlinux.conf` on the Jetson and remove the `console=ttyTCU0,115200` argument from the `APPEND` line. (Be sure to keep a backup of the original configuration).
   * **Disable the systemd Getty Service**: Stop and mask the systemd service to prevent a login shell from spawning on the port:
     ```bash
     sudo systemctl stop serial-getty@ttyTCU0.service
     sudo systemctl disable serial-getty@ttyTCU0.service
     sudo systemctl mask serial-getty@ttyTCU0.service
     ```

## Betaflight CLI Configuration (4.5.x)

For Betaflight 4.5.x, use the following CLI commands to configure high-rate Blackbox streaming while manually stripping unnecessary parameters to reduce the UART payload size:

```bash
feature BLACKBOX
set blackbox_device = SERIAL
set blackbox_mode = ALWAYS

# Disable unwanted data to save bandwidth and simplify parsing
set blackbox_disable_pids = ON
set blackbox_disable_rc = ON
set blackbox_disable_setpoint = ON
set blackbox_disable_bat = OFF      # Enable for Battery Voltage feedback
set blackbox_disable_alt = ON
set blackbox_disable_rssi = ON
set blackbox_disable_debug = ON
# set blackbox_disable_motors = ON
set blackbox_disable_gps = ON
set blackbox_disable_rpm = OFF      # Enable for RPM feedback
set blackbox_disable_attitude = ON  # Replaces 'mag' in newer versions

# Set the Blackbox rate (assuming 8kHz PID loop)
set blackbox_sample_rate = 1/1  # Target 2kHz on 8kHz loop (or 1/1 for max rate)
set blackbox_rate_num = 1
set blackbox_rate_denom = 4

# Configure UART1 (Serial 0) for 1.5Mbps for Blackbox
# Note: While MSP can be enabled, Betaflight prioritizes Blackbox when armed.
serial 0 128 1500000 57600 115200 1500000

save
```

## 🎯 PID & Rates Configuration for Autonomous Control

When deploying autonomous controllers (e.g. RL or MPC) that command angular rates (deg/s) at high frequency (100Hz+), the flight controller's internal rate-tracking loop must be configured to act as a clean, linear actuator.

### 1. Disable Feedforward (FF)
By default, Betaflight uses Feedforward to speed up stick response for human pilots. However, step updates from a 100Hz companion computer will cause extreme motor spikes, motor heating, and potential desyncs.

Disable feedforward on all axes in the CLI using the `f_` prefix:
```bash
set f_roll = 0
set f_pitch = 0
set f_yaw = 0
```

### 2. Configure Linear Rates
The translation bridge (`msp_bridge`) assumes a linear mapping between high-level rate commands and RC channel stick deflections. You must disable expo and configure equal center/max rates.

In the **`ACTUAL`** rate model, rates are configured in **units of 10 deg/s** (e.g. a value of `50` represents $500\text{ deg/s}$).
Apply these commands to configure a linear $500\text{ deg/s}$ rate curve matching the default `max_rate_deg_s: 500.0` parameter in `msp_params.yaml`:
```bash
set rates_type = ACTUAL
set roll_rc_rate = 50
set roll_srate = 50
set roll_expo = 0
set pitch_rc_rate = 50
set pitch_srate = 50
set pitch_expo = 0
set yaw_rc_rate = 50
set yaw_srate = 50
set yaw_expo = 0
```

### 3. Blackbox Bandwidth & Arming Sequence
*   **Avoid Serial Overflows**: Do not set `blackbox_sample_rate = 1/1` on an 8kHz loop rate, as the resulting 8kHz telemetry stream requires ~2.16 Mbps, overflowing the 1.5 Mbps UART connection. Set it to `1/4` (2kHz) or `1/8` (1kHz) instead.
*   **Dictionary Locking**: Keep `blackbox_mode = NORMAL`. Because Betaflight only streams the dictionary headers (`H` lines) at the beginning of a logging session, you must **arm the drone** to initiate the stream and allow the ROS `blackbox_driver_node` to lock onto the frame format definitions.
*   **Allow Tilted Arming**: Set `small_angle = 180` to prevent arming failures or accidental disarms if the drone is tilted during takeoff or aggressive maneuvers.

---

## 🛠️ Building Custom Betaflight Firmware (with CRC-8 Protection)

To secure high-speed serial telemetry and eliminate transient UART noise or gyro spikes, we use a custom Betaflight firmware that appends a frame-level CRC-8 checksum to every Intraframe (I) and Interframe (P) packet. This custom firmware is maintained on a dedicated branch.

### 1. Prerequisites & Toolchain Setup

To compile Betaflight, you need the standard GNU ARM Embedded Toolchain. You can install it on your Ubuntu host using the Betaflight repository scripts:

```bash
cd ~/proj/betaflight
make arm_sdk_install
```

This will automatically download and install the registered Arm SDK/toolchain.

### 2. Fetch the Code & Checkout the Branch

The custom source code containing the CRC-8 stream protection changes is maintained on the `feature/blackbox-crc-validation` branch:

```bash
cd ~/proj/betaflight
git checkout feature/blackbox-crc-validation
```

### 3. Compilation Guide

Compile the appropriate flight controller target for your drone. For example, if you are using an STM32H743-based FC:

```bash
# Clean previous builds
make TARGET=STM32H743 clean

# Compile the target firmware (using multiple cores)
make TARGET=STM32H743 -j$(nproc)
```

Alternatively, to test or compile locally for the software-in-the-loop simulator:
```bash
make SITL -j$(nproc)
```

The resulting binary (`.hex` or `.bin`) will be output under the `obj/` directory (e.g., `obj/betaflight_2026.6.0_STM32H743.hex`).

### 4. Flashing the Firmware

Connect the flight controller to your host via USB and flash the compiled binary.

#### Method A: via Betaflight Configurator
1. Open the Betaflight Configurator.
2. Go to **Firmware Flasher**.
3. Click **Load Firmware [Local]** and select the `.hex` file from your `obj/` directory.
4. Click **Flash Firmware**.

#### Method B: via CLI DFU Mode
If your FC is detected in DFU mode, you can flash directly from the command line:
```bash
make TARGET=STM32H743 dfu_flash
```

---

### 🏗️ Deployment Architecture

The system utilizes a distributed ROS 2 graph to achieve low-latency perception and high-frequency control. The following diagram illustrates the data flow between hardware producers, processing nodes, and the flight controller.

```mermaid
graph TD
    subgraph "External Producers"
        Vicon[Vicon System] -- "Ethernet (UDP)" --> ViconNode
        Radio[Manual Radio/Joy] -- "USB/HID" --> Bridge
    end

    subgraph "Jetson Orin NX (ROS 2 Stack)"
        direction TB
        BB[blackbox_driver_node] -- "/imu0 (2kHz)" --> Ctrl
        ViconNode[vicon_client] -- "Pose/Odom" --> Ctrl
        
        Ctrl[drone_controller] -- "/drone_attitude_cmd" --> Bridge
        JoySub[/joy] -- "Override Toggle" --> Bridge
        
        Bridge[msp_bridge]
    end

    subgraph "Betaflight Flight Controller"
        FC[STM32 FC] -- "UART1 (1.5Mbps)" --> BB
        Bridge -- "USB-VCP (MSP)" --> FC
    end

    style Bridge fill:#f96,stroke:#333Msg,stroke-width:2px
    style FC fill:#bbf,stroke:#333,stroke-width:2px
    style Ctrl fill:#dfd,stroke:#333,stroke-width:2px
```

### Data Flow Logic
1.  **Perception**: High-rate IMU data is streamed via UART1 and processed by the C++ `blackbox_driver_node`. This node identifies 'I' and 'P' frames to reconstruct the state.
2.  **State Estimation**: The `drone_controller` fuses IMU telemetry with Vicon ground truth (or Gate Detection) to compute optimal actuator commands.
3.  **Control Injection**: The `msp_bridge` node acts as a virtual RX. It consumes attitude rate commands and injects them into the FC via the MSP protocol over USB, bypassing the serial priority constraints of the primary UART.

For detailed information on the Blackbox Driver configuration and ROS 2 parameters, see the [Blackbox Driver](perception/blackbox.md) page.

---

## 🚀 System Launch & Configuration

The primary entry point for the flight stack is the `racing_bringup.launch.py`. This launch file orchestrates the initialization of all hardware interfaces and control logic.

### Standard Execution
```bash
ros2 launch drone_control racing_bringup.launch.py
```

### Launch Arguments & Options

| Argument | Default | Description |
| :--- | :--- | :--- |
| `shared_port` | `/dev/ttyTHS1` | Primary UART for high-rate Blackbox telemetry. |
| `rc_sim_port` | `/dev/ttyACM0` | USB-VCP port for the MSP Control Bridge. |
| `use_rc_sim` | `true` | Enables the `msp_bridge` node for autonomous control. |
| `use_vicon` | `true` | Enables the Vicon receiver for indoor ground truth. |
| `use_mavsdk` | `false` | Enables MAVSDK (Requires MAVLink-to-MSP gateway). |

### Overriding Parameters
You can override hardware ports or toggle features directly from the CLI:
```bash
ros2 launch drone_control racing_bringup.launch.py rc_sim_port:=/dev/ttyUSB0 use_vicon:=false
```

---

## 🛠️ Operational Modes & Safety

### Manual Override (AUX2)
The `msp_bridge` node respects a safety switch mapped to **AUX2**. 
*   **Manual Mode (AUX2 LOW)**: The `msp_bridge` passes through joystick/radio inputs to the flight controller. This is the recommended state for take-off and landing.
*   **Autonomous Mode (AUX2 HIGH)**: The node ignores manual stick inputs and injects commands from the ROS `/drone_attitude_cmd` topic.

### Monitoring & Diagnostics
The `msp_bridge` provides a periodic heartbeat log at **1Hz** to confirm active command injection:
```text
[INFO] [msp_bridge]: Sending MSP RC: R:1500 P:1500 T:1140 Y:1500 A1:2000 A2:2000 (OVERRIDE)
```
If the indicator shows `(MANUAL)`, the drone will not respond to autonomous commands even if the node is running.

---

## 🛡️ Troubleshooting Refined

If the `msp_bridge` fails to connect:
1.  Check the USB connection to the Betaflight FC. It must appear as `/dev/ttyACM0`.
2.  Ensure the user is in the `dialout` group: `sudo usermod -aG dialout $USER`.
3.  Verify the port isn't locked by another process (e.g., Betaflight Configurator).
