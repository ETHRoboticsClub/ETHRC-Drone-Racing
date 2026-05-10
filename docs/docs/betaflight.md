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
   > This setting is volatile. It MUST be reapplied after every Jetson reboot.

3. **Power Mode**: Run the Jetson in `MAXN` mode for stable high-rate serial processing:
   ```bash
   sudo nvpmodel -m 0
   sudo jetson_clocks
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
