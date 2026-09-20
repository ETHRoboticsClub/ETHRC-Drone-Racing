# Boreas (4S) Setup

**Page Owner:** [Dimetri Chau](https://github.com/deonixlive)

*4S drone with a JHEMCU generic FC (H743 chip).*

## 🛠️ Hardware & Parts List

| Component | Specification |
| :--- | :--- |
| **Frame** | TBS Source Two v0.1 |
| **Flight Controller** | JHEMCU Generic H743 |
| **Motors** | OrqaFPV 2207 (4S, unknown kv) |
| **ESC** | OrqaFPV 3030 ESC (Supports 80A and AM32) |
| **Propellers** | 5-inch Gemfan or OrqaFPV |
| **Receiver** | ELRS or Ghost (GHST) system |
| **FPV System** | Analog possible (currently not equipped) |
| **Cost** | [TBD] |

## ⚡ Soldering & Connections
[TBD: Wiring diagrams or descriptions]

## 🎯 Betaflight Configuration

To configure **Boreas** (running Betaflight 4.5.3+) for high-rate IMU Blackbox streaming (UART6) and MSP RC simulation override / MSP Bench (UART1), apply the following commands in the Betaflight CLI:

```text
# 1. Configure Ports (UART1 = MSP @ 1.5M, UART6 = Blackbox @ 1.5M)
# Port 0 (UART1) is configured for MSP (1) with 1.5M baud
# Port 5 (UART6) is configured for Blackbox (128) with 1.5M baud
serial 0 1 1500000 57600 0 115200
serial 5 128 115200 57600 0 1500000

# 2. Configure Blackbox Settings
set blackbox_device = SERIAL
set blackbox_mode = NORMAL
set blackbox_sample_rate = 1/16

# Disable high-bandwidth parameters to optimize sample parsing (reduces serial load)
set blackbox_disable_pids = ON
set blackbox_disable_rc = ON
set blackbox_disable_setpoint = ON
set blackbox_disable_bat = OFF      # Keep enabled for voltage feedback
set blackbox_disable_alt = ON
set blackbox_disable_rssi = ON
set blackbox_disable_debug = OFF     # Keep enabled to record debug data (RPM telemetry)
set blackbox_disable_motors = OFF    # Keep enabled to record motor commands
set blackbox_disable_rpm = OFF       # Keep enabled to record motor RPM
set blackbox_disable_gps = ON
set debug_mode = DSHOT_RPM_TELEMETRY # Enable DShot RPM debug mode for motor logging

# 3. Configure MSP Bench Mode & RC Simulation Override
set msp_override_channels_mask = 15
set msp_override_failsafe = ON

# Save and reboot the FC
save
```

> [!NOTE]
> **Betaflight 4.5+ Syntax Deprecations:**
> - `feature BLACKBOX` is deprecated (Blackbox is always built-in and active).
> - `blackbox_rate_num` and `blackbox_rate_denom` are deprecated (replaced entirely by `blackbox_sample_rate` e.g. `1/16`).
> - `blackbox_disable_attitude` is removed in this target build; only standard sensor disables are needed.
> - Omit the `115200` telemetry parameter in the `serial` command (use `0` instead) to avoid parser `INVALID ARGUMENT COUNT` errors.

## 📦 Specifics & Precompiled Images
[TBD: Custom firmware builds or images]
