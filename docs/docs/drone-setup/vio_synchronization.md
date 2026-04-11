# VIO & IMU Synchronization (Sub-Microsecond)

To achieve stable state estimation (VIO) at the speeds required for autonomous drone racing (>20 m/s), high-precision synchronization between the IMU and the camera frame start is non-negotiable. This document details the "Hard-Sync" architecture using the Jetson Hardware Timestamping Engine (HTE).

## 🏗 The Hard-Sync Architecture

Instead of relying on soft-timestamps (Argus arrival time), we use a dedicated electrical pulse to correlate the Flight Controller (FC) and Jetson clocks.

### 1. External Trigger (Kakute H7)
We repurpose the **M6** (MAIN6) motor pin on the Kakute H7 v1.5 as a camera trigger.
- **Signal**: 3.3V Pulse (High for 1ms) per frame integration start.
- **Firmware (PX4)**:
    - Set `TRIG_INTERFACE` to `1` (GPIO).
    - Set `TRIG_PINS` to `6` (remapping M6).
    - Set `TRIG_MODE` to `1` (triggered on integration).

### 2. Hardware Timestamping (Jetson Orin NX)
The pulse from M6 is connected to an **Always On (AON)** GPIO on the Jetson Orin NX header.
- **Target Pin**: **Pin 31** (SoC: `GPIO3_PAA.00`).
- **Feature**: Hardware Timestamping Engine (HTE) in JetPack 6.2.
- **Clock**: The HTE timestamps the rising edge in the `TSC` (Time Stamp Counter) domain, providing sub-microsecond accuracy independent of CPU load or interrupt latency.

---

## 🛠 Hardware Wiring

| Connection | Kakute H7 (FC) | Seeed A603 (Jetson) | Notes |
| :--- | :--- | :--- | :--- |
| **Sync Signal** | M6 (Signal Pin) | Header Pin 31 | 3.3V Logic Level |
| **Ground** | ESC Ground | Header Pin 39 | Common reference is critical |

> [!WARNING]
> Ensure the Flight Controller and Jetson share a common ground. If powered by separate regulators, verify there is no ground loop.

---

## 🏁 High-Rate Betaflight Integration (Betaflight)

For ultra-low latency configurations, we support a 2000Hz IMU stream using the Betaflight Serial Blackbox methodology.

### 1. External Trigger (Betaflight)
We repurpose a PinIO-capable pad (e.g., **LED_STRIP**) to generate a sync pulse.
- **Signal**: 3.3V Pulse (High for 1ms) triggered by `USER1` mode.
- **Betaflight CLI**:
    ```bash
    resource LED_STRIP 1 NONE
    resource PINIO 1 <PAD_RESOURCE>
    set pinio_config = 1,1,1,1
    set pinio_box = 40,255,255,255
    save
    ```

### 2. Hardware Timestamping (Pin 16)
The Betaflight sync pulse is connected to **Pin 16** (SoC: `GPIO3_PBB.01`) on the Jetson Orin NX.
- **HTE**: This pin is part of the AON domain and supports raw hardware timestamping via the HTE peripheral.

---

## 💻 Software Configuration

### 1. Device Tree Patch (JETSON-SIDE)
To enable HTE on Pin 31, you must patch the device tree to include the HTE test driver or a custom VIO-Sync driver:

```dts
// Fragment for tegra-hte-test
tegra_hte_test {
    status = "okay";
    gpio-controller = <&tegra_aon_gpio>;
    // Map to PAA.00 (Pin 31)
    hte-gpios = <&tegra_aon_gpio TEGRA234_AON_GPIO(AA, 0) GPIO_ACTIVE_HIGH>;
};
```

### 2. Clock Correlation Logic
The VIO node (OpenVINS / Rovioli) should consume:
1.  **IMU Samples**: Received via Micro-XRCE-DDS with FC timestamps.
2.  **HTE Pulses**: Extracted from `/dev/hte_test` (or kernel events).
3.  **Camera Frames**: Received via Argus.

**Synchronization math**:
$$T_{offset} = T_{HTE\_Timestamp} - T_{FC\_Trigger\_Msg}$$
This offset is tracked using a Kalman Filter to account for clock drift between the STM32H7 (FC) and the Orin NX.

---

## ⏰ System Time Synchronization (Jetson Orin NX)

Before performing any hardware-triggered synchronization, ensure the Jetson system clock is aligned with the ground station/capture host. This is critical for TLS/Docker stability and long-term clock drift compensation.

### Tool: `sync_time.sh`
Use the provided utility to synchronize the `boreas` clock with your local ground station:
```bash
./scripts/tools/remote/sync_time.sh
```
This script captures the current UTC time from the GS and applies it to the remote Jetson via SSH, bypassing issues with volatile RTCs on developer hardware.

---

## 📚 Technical Sources
- **ArXiv 2601.15222**: Highlights the "Trigger-based Hardware Sync" as the primary reason for stable VIO at high-alpha maneuvers.
- **NVIDIA HTE Guide**: Documentation for [Hardware Timestamping in JP6](https://docs.nvidia.com/jetson/archives/r36.2/DeveloperGuide/SD/CameraDevelopment.html#hardware-timestamping).
- **PX4 User Guide**: [Camera Trigger Configuration](https://docs.px4.io/main/en/peripherals/camera.html).

---
**Branch**: `feature/timesync`
