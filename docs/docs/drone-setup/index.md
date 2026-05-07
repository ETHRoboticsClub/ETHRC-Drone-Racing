# Drone Setup & Hardware

**Page Owner:** [Dimetri Chau](https://github.com/deonixlive)

---

Welcome to the Drone Setup documentation. This section covers our real-world racing hardware, electronics, and mechanical design.

## 🌬️ Meet the Wind Gods

We name our racing fleets after the Greek wind gods to reflect their agility and speed.

| Drone | Class | Flight Controller | Purpose |
| :--- | :--- | :--- | :--- |
| **[Zephyrus](zephyrus.md)** | 6S High Performance | Kakute H7 | Primary high-speed racing platform. |
| **[Boreas](boreas.md)** | 4S Agile | JHEMCU Generic H743 | Lightweight, agile platform for technical tracks. |

## 🛠️ Calibration & Tuning

*   **[Jetson Camera Pipeline](../perception/camera_pipeline.md)**: Current IMX219 60 Hz perception and H.264 compression path.
*   **[IMX219 90Hz High-Speed Tuning](imx219_90hz_tuning.md)**: Optimizing the camera for high-rate perception.
*   **[VIO Synchronization](vio_synchronization.md)**: Hardware-level sync between the IMU and perception stack.
*   **[Betaflight Config (2kHz Blackbox)](../betaflight.md)**: Detailed Betaflight CLI and tuning settings.
