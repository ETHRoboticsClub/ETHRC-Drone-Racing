# Betaflight Manual Configuration

**Page Owner:** [Mattia De Martino](https://github.com/mattiademartino)

When switching between **Flight Mode** and **MSP Bench Mode**, it is necessary to update the Flight Controller's configuration. While we provide automated scripts for this, manual loading via the **Betaflight Configurator CLI** is the most reliable method.

---

## 1. Entering Bench Mode
Bench mode disables the physical RC receiver and enables MSP overrides, allowing you to control the drone via the `msp_tx_emulator`.

1.  Connect your Flight Controller to your PC via USB.
2.  Open **Betaflight Configurator** and click **Connect**.
3.  Go to the **CLI** tab.
4.  Open the file [ZEPHYRUS_MSP_BENCH.txt](../files/betaflight_conf/ZEPHYRUS_MSP_BENCH.txt) in your editor.
5.  Copy the entire content of the file.
6.  Paste it into the Betaflight CLI text box and press **Enter**.
7.  The FC will process the commands and should automatically `save` and reboot.

> [!TIP]
> After rebooting, check the **Receiver** tab. If you run the `msp_tx_emulator` on the Jetson, you should see the bars moving even with your physical transmitter turned off.

---

## 2. Reverting to Flight Mode
To fly the drone normally with a physical transmitter, you must restore the original flight configuration.

1.  Connect to the **CLI** tab in Betaflight Configurator.
2.  Open the file [zephyrus_KAKUTEH7.txt](../files/betaflight_conf/zephyrus_KAKUTEH7.txt).
3.  Copy and paste the content into the CLI.
4.  Press **Enter** and wait for the reboot.

---

## 3. Key Troubleshooting Settings
If you are modifying these configurations manually, ensure the following critical settings are applied for Bench Mode:

| Command | Description |
| :--- | :--- |
| `set msp_override_channels_mask = 65535` | Enables MSP control for all 16 channels (including Arm). |
| `set msp_override_failsafe = ON` | Prevents failsafe when physical RX is missing. |
| `set small_angle = 180` | Allows arming even if the drone is not level on the bench. |
| `feature -RX_SERIAL` | Disables the physical serial receiver. |

---

## 4. Why Manual Loading?
Serial communication over UART/USB to the Betaflight CLI can be timing-sensitive. The automated `load_bf_config` script may occasionally fail to "wake up" the CLI or miss the `save` command depending on the specific FC firmware and system load. Using the Configurator CLI ensures that every command is processed and confirmed.
