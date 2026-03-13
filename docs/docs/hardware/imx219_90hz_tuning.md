# IMX219 90Hz High-Speed Tuning

This guide provides a professional procedure to enable **90Hz/92Hz** capture with a **Full 155° FOV** on the Jetson Orin NX. 

> [!NOTE]
> **Hardware Context**: While this solution was developed and verified using the **Seeed Studio A603 carrier board**, the core sensor and timing modifications are applicable to any standard Jetson Orin NX setup. Users with different carrier boards should primarily verify the Device Tree pinmux and clock assignments.

> [!IMPORTANT]
> **Environment Tested**: This solution has been verified on **Ubuntu 22.04 LTS** running **JetPack 6.2 (L4T R36.4.3)**. It assumes you are working within the standard NVIDIA Linux for Tegra (L4T) source tree on a host PC or directly on the Jetson.

## 📋 Prerequisites & Filesystem
We assume a standard L4T directory structure. Let `$L4T_DIR` be your base directory (e.g., `~/nvidia/nvidia_sdk/JetPack_6.2_Linux_JETSON_ORIN_NX_TARGETS/Linux_for_Tegra/`).

### 1. Extracting Sources
If you haven't already, extract the Out-of-Tree (OOT) driver sources provided by NVIDIA:
```bash
cd $L4T_DIR/source
tar -xjf kernel_oot_modules_src.tbz2
# This creates the 'nvidia-oot' directory
```

### 2. Tools
- Ensure `dtc` (Device Tree Compiler) is installed: `sudo apt install device-tree-compiler`.

---

## 🛠 Step 2: Sensor Driver Patches
You must modify the registration tables in the `nvidia-oot` driver to align the physical sensor timing with 90Hz logic.

- **File Path**: `source/nvidia-oot/drivers/media/i2c/imx219_mode_tbls.h`

### 2.1 Timing & Exposure Guard
Modify the `imx219_mode_1280x720_60fps` table (which we repurpose for 90Hz):
- **Exposure (592 lines)**: Set `{0x015A, 0x02}` and `{0x015B, 0x50}`.
- **VMAX (633 lines)**: Set `{0x0160, 0x02}` and `{0x0161, 0x79}`.
- **Line Length (4000 lines)**: Set `{0x0162, 0x0F}` and `{0x0163, 0xA0}`.

### 2.2 Full-FOV Reduction (4x4)
To map the full 155° sensor area instead of a crop, combine Analog Binning with Subsampling:
- **X/Y_ODD_INC (2x2 Skip)**: `{0x0170, 0x03}`, `{0x0171, 0x03}`.
- **BINNING_MODE (2x2 Analog Bin)**: `{0x0174, 0x03}`, `{0x0175, 0x03}`.

### 2.3 CSI Protocol Stability
- **Data Format (RAW10)**: Ensure `{0x018C, 0x0A}` and `{0x018D, 0x0A}`.

---

## 🏗 Step 3: Device Tree Turbo-Drain
The Jetson's Video Input (VI) pipeline must drain the CSI FIFO faster than the sensor's output to prevent overflows at high framerates.

1. Locate the `imx219@10` node in your carrier board's DTS (e.g., `tegra234-p3768-0000-a0.dts`).
2. Update the `pix_clk_hz` property:
```dts
// Force 400MHz host drainage priority
pix_clk_hz = "400000000"; 
```

---

## 🚀 Step 4: Build & Deploy
1. **Compile**: Navigate to the `nvidia-oot` root and run `make modules`. 
2. **Install**: 
   ```bash
   # Copy the compiled module to the target updates directory
   sudo cp nv_imx219.ko /usr/lib/modules/$(uname -r)/updates/drivers/media/i2c/
   sudo depmod -a
   ```
3. **Reboot**: A full power cycle is recommended to clear the sensor logic and apply the new Device Tree configuration.

---

## ⚠️ Known Pitfalls & Troubleshooting
| Symptom | Probable Cause | Fix |
| :--- | :--- | :--- |
| **Black Screen** | Exposure > Frame Length | Ensure Exposure = 592 and VMAX = 633. |
| **CSI Timeout** | Protocol Mismatch | Verify `0x018C/D` is `0x0A` (RAW10). |
| **Quarter FOV** | Binning Only | Ensure BOTH Binning AND Skipping bits are set to `0x03`. |
| **FIFO Overflow** | Low Host Clock | Verify `pix_clk_hz` is strictly `400000000`. |

## 📚 Technical Sources
- **Inspiration**: [ArXiv 2601.15222](https://arxiv.org/abs/2601.15222) - Drone Racing Perception.
- **Hardware Reference**: [IMX219 Datasheet](https://www.arducam.com/downloads/modules/RaspberryPi_camera/IMX219DS.PDF).
- **Driver Reference**: NVIDIA Tegra [Camera Development Guide](https://docs.nvidia.com/jetson/archives/r36.2/DeveloperGuide/SD/CameraDevelopment.html).
