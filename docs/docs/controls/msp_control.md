# Betaflight MSP Control Architecture

**Page Owner:** [Mattia De Martino](https://github.com/mattiademartino)

This section describes how the ETHRC Drone Racing framework bridges autonomous high-level control commands (such as those from an MPC or RL agent) directly into the Betaflight flight controller.

## Introduction to Hardware-in-the-Loop Control

For autonomous flight, the companion computer (NVIDIA Jetson) must be able to inject flight commands into the drone's flight controller, substituting the pilot's traditional remote control sticks. 

While many robotics autopilots (like PX4 or ArduPilot) utilize **MAVLink** for receiving these commands, **Betaflight** operates natively on the **Multiwii Serial Protocol (MSP)**. Therefore, the most native, low-latency, and reliable method to control a Betaflight drone via a companion computer is to stream MSP packets directly over a serial connection. 

This architecture allows an external computer to "override" the pilot's stick inputs seamlessly while keeping the pilot in the loop with a physical safety switch.

---

## Software Architecture

The control pipeline from the autonomous planner down to the flight controller consists of two ROS 2 nodes connected in sequence:

1. **`betaflight_msp_controller` (The Translation Bridge)**
2. **`multiwii2_node` (The Serial Driver)**

### 1. `betaflight_msp_controller` (Translation Bridge)

The high-level controllers (e.g., `drone_controller_node`) output standardized robotics commands—specifically thrust values and rotational rates (roll, pitch, yaw in degrees per second)—to the `/drone_attitude_cmd` topic. 

Betaflight, however, expects these values as Pulse Width Modulation (PWM) signals ranging from `1000` to `2000` on specific channels (Aileron, Elevator, Rudder, Throttle). The `betaflight_msp_controller` node subscribes to the continuous stream of attitude commands and mathematically maps them into 1000-2000 PWM signals.

**Configurable Parameters:**
* `attitude_cmd_topic` *(string)*: The input topic name to listen to. **Default:** `'/drone_attitude_cmd'`
* `rc_override_topic` *(string)*: The output topic where the `OverrideRCIn` message will be published. **Default:** `'/multiwii/rc/override'`
* `max_rate_deg_s` *(float)*: The maximum rotational rate (in degrees per second) that corresponds to maximum stick deflection (a PWM of 2000 or 1000). **Default:** `500.0`. 
  * *Example:* If `max_rate_deg_s` is 500, an incoming roll rate command of `250.0` translates mathematically to a center-offset PWM of `1750`.

### 2. `multiwii2_node` (Serial Driver)

Once the commands are accurately mapped to the 1000-2000 range, they are sent as `mavros_msgs/msg/OverrideRCIn` messages to the `multiwii_ros2` node. This open-source C++ node is responsible for managing the actual serial hardware (`/dev/ttyACM0` via USB/UART). It converts the structured ROS message into an `MSP_SET_RAW_RC` binary packet and transmits it over the serial lines to the flight controller.

---

## Betaflight FC Configuration

To enable this architecture, the flight controller must be running Betaflight version **4.5.2 or higher** (4.6 is officially recommended by high-performance MSP libraries like [Deonixlive](https://github.com/Deonixlive/Control-Betaflight-Copter)) and configured to trust the Jetson's manual serial inputs.

### 1. Enable MSP on the Port
Connect the Jetson to the Flight Controller (typically via an available UART or the USB-C connection exposing `/dev/ttyACM0`). In the **Betaflight Configurator** > **Ports** tab, ensure that the **Configuration/MSP** toggle is enabled for the port physically connected to the Jetson. If you're using high update frequencies, use a high Baud rate (e.g., `1000000` or `115200`).

### 2. Configure the MSP Channels Mask in CLI
Betaflight strictly limits which radio channels an external computer can override for safety. You must manually unlock the Roll, Pitch, Yaw, and Throttle channels by defining the `msp_override_channels_mask`.

The bits in the mask correlate to: `AUX16 | ... | AUX2 | AUX1 | YAW | THROTTLE | PITCH | ROLL`.
Setting a bit to `1` enables the override.

* **Warning:** It is highly advised *against* overwriting `AUX1` because the ARMING switch is typically located there. Let the human pilot retain arming authority!

Open the **CLI** tab in Betaflight Configurator and paste one of the following commands based on your needs:

#### Common Override Examples

**1. Standard Autonomous Flight (Roll, Pitch, Yaw, Throttle)** 
This is the most common setup. It grants the Jetson full directional control.
```bash
set msp_override_channels_mask = 15
save
```

**2. Throttle Test Mode (Throttle Only)**
Useful when initially tuning the PID hover values. The Jetson only controls altitude (Throttle = Bit 3 = 4), while you maintain standard Roll, Pitch, and Yaw via your remote.
```bash
set msp_override_channels_mask = 4
save
```

**3. Flight + Payload Drop (AETR + AUX2)**
If the Jetson also needs to trigger a servo or payload drop mechanism located on `AUX2` (Bit 6 = 32). `32 + 15 = 47`.
```bash
set msp_override_channels_mask = 47
save
```

*(Tip: If you notice during testing that the Yaw and Throttle seem reversed, verify that Betaflight hasn't swapped the internal YAW/THROTTLE channel order inside your specific version's MSP parser).*


### 3. Configure MSP Override (Safety Switch)
To ensure the pilot maintains ultimate authority over the drone, we utilize Betaflight's built-in **MSP Override** mode.

1. Navigate to the **Modes** tab in the Betaflight Configurator.
2. Locate the **MSP OVERRIDE** mode.
3. Assign this mode to a physical switch on your RC Transmitter (e.g., `AUX2` or `AUX3`).

**How it works during a flight:**
* **Switch OFF (Manual Mode):** Betaflight strictly listens to your radio receiver. The Jetson is continuously generating and sending autonomous flight commands into the void; Betaflight safely ignores them.
* **Switch ON (Autonomous Mode):** Betaflight seamlessly transfers control of the Roll, Pitch, Yaw, and Throttle channels (due to our mask of `15`) over to the continuous stream of `MSP_SET_RAW_RC` packets provided by the Jetson. 
* If the drone begins to act erratically, the human pilot immediately flips the switch back to OFF, instantly regaining manual control.

---

## Advanced: MSP Extensions (Controlling More)

While `MSP_SET_RAW_RC` is sufficient for standard flight stick emulation, Betaflight also supports a suite of broader **[MSP Extensions](https://www.betaflight.com/docs/development/API/MSP-Extensions)**. 

Should we ever need to "control more" than just the flight axes, these extensions allow the Jetson to dig deeper into the actual state machine and configuration of the flight controller in real-time. For example:

* **`MSP_SET_MODE_RANGE`**: Instead of relying on the human pilot to pre-configure physical AUX switches to trigger functions like Angle Mode, Acro Trainer, or Beeper, the Jetson can programmatically redefine mode ranges on the fly.
* **`MSP_SET_ADJUSTMENT_RANGE`**: If the high-level Reinforcement Learning (RL) or MPC node determines that the drone's PID tuning is no longer optimal due to battery sag or turbulence, it can use MSP adjustment ranges to dynamically push changes to the PID loops mid-flight.
* **Other Integrations**: Broader MSP commands also allow the companion computer to push custom text directly to the pilot's OSD (On-Screen Display), change Video Transmitter (VTX) power levels, or change LED Strip configurations based on the Jetson's perception state.

*To utilize these advanced extensions in the future, the `multiwii2_node` or a direct serial driver would need to be expanded to accept and serialize these specific Betaflight API commands.*

---

## Running the Pipeline

There are two primary ways to run the control pipeline: via individual nodes or using the unified "Racing Bringup" script.

### 1. Unified Racing Bringup (Recommended)
This script starts the entire stack (Comms, Perception, and Control) in one go. It is the fastest way to get to a flight-ready state.

```bash
ros2 launch drone_control racing_bringup.launch.py msp_port:=/dev/ttyTHS1
```

### 2. Manual Node Launch
If you need more granular control or are debugging a specific component, you can launch the MSP bridge individually:

```bash
# Launch with default YAML parameters (ttyTHS1, 1.5Mbps)
ros2 launch betaflight_msp_controller msp_bridge.launch.py

# Launch with CLI overrides
ros2 launch betaflight_msp_controller msp_bridge.launch.py port:=/dev/ttyUSB0 baudrate:=115200
```

---

## Configuration

The system supports two levels of configuration:

1.  **YAML Parameters**: Defaults are stored in `src/betaflight_msp_controller/config/msp_params.yaml`. Edit this file to set your "permanent" hardware setup.
2.  **CLI Arguments**: Any parameter in the YAML can be overridden at runtime using the `name:=value` syntax in the launch command.

---

## Testing Without a Radio Controller

In development environments where a physical RC Transmitter is unavailable, you can use the `arm_and_override_persistent.py` utility script to simulate the radio link and inject autonomous commands simultaneously.

### Persistent Arm & Override Logic
This script utilizes two separate serial connections to the Flight Controller:
1.  **Primary RX (USB)**: Emulates the RC Receiver, sending continuous heartbeat packets for **ARMing** (AUX1) and enabling **MSP_OVERRIDE** mode (AUX2).
2.  **Override Port (UART)**: Sends the actual AETR stick values (Roll, Pitch, Yaw, Throttle) that will bypass the primary RX while the override switch is active.

### Usage
Run the script from the Jetson (ensure permissions are set for `/dev/ttyACM*` and `/dev/ttyTHS*`):
```bash
python3 scripts/msp/arm_and_override_persistent.py /dev/ttyACM0 /dev/ttyTHS1
```

**Safety Features:**
*   **Initialization**: Sends 1 second of zero throttle/disarmed packets to clear Betaflight's boot safety checks.
*   **Heartbeat**: Maintains a 50Hz (20ms) update rate to prevent failsafes.
*   **Emergency Stop**: Pressing `Ctrl+C` immediately sends a burst of Disarm packets and zeroes all channels before closing the ports.

---

## ⚠️ Potential Pitfalls & Troubleshooting

### 1. Serial Port Permissions
If the node fails to open the serial port with a "Permission Denied" error, your user likely lacks the necessary group permissions.
**Solution:** Add your user to the `dialout` and `tty` groups, then log out and back in.
```bash
sudo usermod -a -G dialout,tty $USER
```

### 2. Package Discovery (Colcon)
The `betaflight_msp_controller` was moved to the top-level `src/` directory to ensure `colcon` finds it. If you move it back inside another package, `ros2 launch` will fail to find it.

### 3. Baud Rate Mismatch
Betaflight's `msp_baudrate` (visible in the Ports tab) **must match** the Jetson's baud rate. If they differ, the node will "connect" but no data will be parsed by the FC.
*   **Default Project Baud:** `1500000` (1.5 Mbps) for high-rate IMU streaming.

### 4. Cable Quality
Always use a high-quality **data** cable. Many USB cables included with consumer electronics are "charging only" and do not have the data lines connected, which will result in the device not appearing in `/dev/`.

### 5. Proper Orientation
If the drone pitches when you command roll, check your **Betaflight Board Alignment** settings and ensure the `max_rate_deg_s` in `msp_params.yaml` matches your high-level controller's expectations.

---

## Testing Without Props
**IMPORTANT:** Always test your MSP override configuration **WITHOUT PROPELLERS**. 
1. Launch the `msp_bridge.launch.py`.
2. Connect the FC to Betaflight Configurator.
3. Use the **Receiver Tab** to verify that sending commands from ROS 2 actually moves the Roll/Pitch/Yaw/Throttle sliders in real-time.
