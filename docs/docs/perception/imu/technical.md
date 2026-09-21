# IMU — technical

**Owner:** [Ilyas Seckin](https://github.com/ilysec)

The drones have no separate IMU. Gyroscope and accelerometer data come from the Betaflight flight controller, streamed over a dedicated UART to the Jetson. Package: `src/perception/betaflight_blackbox_driver`. It contains two executables that publish the same topics from different protocols.

| Executable | Protocol | Serial rate | Notes |
| :--- | :--- | :--- | :--- |
| `blackbox_driver_node` | Betaflight **blackbox** binary log streamed to a serial port | 1.5 Mbaud, ~2 kHz frames | Production (`imu_source:=blackbox`). Requires the custom firmware with per-frame CRC-8. |
| `msp_imu_driver_node` | **MSP** request/response polling via the `msp` library | 115200 baud | Fallback (`imu_source:=msp`); lower rate, no validation |

The package has no launch file or config of its own; `racing_bringup.launch.py` sets the parameters (see [IMU → Workflow](workflow.md)).

## `blackbox_driver_node`

### What it does

1. Reads the UART in a real-time thread (SCHED_FIFO 80 when permitted, `ASYNC_LOW_LATENCY` on the tty) into a 64 KiB ring buffer.
2. Parses the blackbox stream: `H` header lines define the field names, encodings and predictors; `I` frames carry absolute values; `P` frames carry deltas against the previous frame. The decoder implements Betaflight's variable-byte, Tag2_3S32, Tag8_8SVB and NEG_14BIT encodings and the ZERO / PREVIOUS / STRAIGHT_LINE / AVERAGE_2 predictors (any other predictor id falls back to PREVIOUS). MSP v1/v2 packets that share the UART are skipped.
3. Verifies a CRC-8 (DVB-S2) over every I and P frame. This CRC is not part of stock Betaflight; the flight controller must run the firmware described in [Betaflight → custom firmware](../../betaflight.md#building-custom-betaflight-firmware-with-crc-8-protection).
4. Optionally validates each sample (range, jump, zero, sliding-window outlier). A validation failure drops the sample but keeps the delta chain; a decode failure or `max_consecutive_rejects` triggers a resync to the next I frame.
5. Publishes `/imu0` from a wall timer at `target_hz` using the latest accepted sample. The IMU rate is therefore constant and independent of the ~2 kHz serial rate; intermediate samples are discarded, not averaged. RPM, motor and RC topics are published for **every** accepted frame, i.e. at the serial rate; battery every 40th frame.

Field names are matched by name, so the Betaflight field set can change without code changes. Gyro prefers `gyroUnfilt`/`gyroRaw`/`gi` over `gyroADC`; accel prefers `accelRaw`/`ai` over `accelADC`/`accSmooth`. Filtered fields, when present, go to a second topic.

### Units

| Quantity | Raw | Published |
| :--- | :--- | :--- |
| Acceleration | 2048 LSB per g (`acc_1G`) | m/s², `raw / 2048 × 9.80665` |
| Angular rate | deg/s (already scaled by the FC) | rad/s |
| eRPM | eRPM / 100 | mechanical RPM, `raw × 100 / (motor_poles / 2)` |
| Battery | `vbatLatest` | volts, `raw × vbat_scale` (0.01) |

Axes are published exactly as Betaflight reports them, `frame_id: imu_link`. No orientation is estimated (`orientation_covariance[0]` is left at 0 — consumers should not read the orientation field).

### Published topics

QoS on all sensor topics: best-effort, keep-last 1.

| Topic (parameter) | Type | Rate | Content |
| :--- | :--- | :--- | :--- |
| `/imu0` (`imu_topic`) | `sensor_msgs/Imu` | `target_hz` | Unfiltered gyro + accel |
| `/imu_filtered` (`imu_filtered_topic`) | `sensor_msgs/Imu` | `target_hz` | Betaflight-filtered gyro (+ accel if logged); only if the header lists filtered fields |
| `/motor_rpm` (`rpm_topic`) | `std_msgs/Float64MultiArray[4]` | frame rate (~2 kHz) | Mechanical RPM; needs bidirectional DShot and eRPM fields in the header |
| `/battery` (`battery_topic`) | `sensor_msgs/BatteryState` | every 40th frame (~50 Hz) | Voltage; `health = DEAD` below `min_battery_voltage` |
| `/motor_commands` (`motor_topic`) | `std_msgs/Float64MultiArray[4]` | frame rate (~2 kHz) | Betaflight `motor[]` outputs |
| `/joystick_inputs` (`joystick_topic`) | `std_msgs/Float64MultiArray[4]` | frame rate (~2 kHz) | `rcCommand[]` |
| `/blackbox_diagnostics` (`diagnostics_topic`) | `diagnostic_msgs/DiagnosticArray` | 0.5 Hz | Byte/frame/reject counters, resyncs, battery state |

### Parameters

| Parameter | Default | Meaning |
| :--- | :--- | :--- |
| `port`, `baud` | `/dev/ttyTHS1`, `1500000` | UART. At ≥ 1 Mbaud the node checks the Jetson `uarti` clock and exits if it is below 80 MHz |
| `target_hz` | 150 | Publish rate, clamped to 100–200 |
| `use_blackbox_time_for_stamp` | false | Stamp with the FC's µs clock (offset to ROS time on first frame, monotonic) instead of the publish timer |
| `validate_imu_stream` | true | Enable range/jump/outlier validation. **The bringup sets this to `false`.** |
| `reject_zero_imu` | true | Drop frames where all six axes are exactly zero |
| `max_frame_delta_us` | 10000 | Max accepted P-frame time step |
| `max_gyro_deg_s`, `max_accel_g` | 4000, 32 | Absolute range limits (`max_gyro_raw_deg_s` is a deprecated alias) |
| `max_gyro_jump_deg_s`, `max_accel_jump_g` | 1000, 8 | Per-frame jump limits |
| `max_consecutive_rejects` | 5 | Rejects before a forced resync |
| `outlier_sigma_k` | 4.0 | Sliding-window (8 samples) outlier threshold, σ floor 10 LSB |
| `motor_poles` | 14 | Overridden by `H motor_poles:` in the header when present |
| `vbat_scale`, `min_battery_voltage` | 0.01, 14.0 | Battery scaling and warning threshold |
| `debug_raw` | false | Dump the raw UART stream to `/tmp/capture.bbl` |
| `enable_msp_control` | false | See below |
| `attitude_command_topic` | `/drone_attitude_cmd` | Input for the MSP control path |

### MSP control path

With `enable_msp_control:=true` the node also **transmits** on the same UART: it subscribes to `attitude_command_topic` (`Float64MultiArray` `[roll_rate, pitch_rate, yaw_rate, thrust]`, rates in deg/s scaled by `max_rate_deg_s = 500`, thrust 0–1), converts to RC channels and sends `MSP_SET_RAW_RC` with AUX1 = 2000 (armed) and AUX2 = 1000. A 100 ms watchdog disarms and lowers throttle if no command arrives for 0.5 s. This is a shared-UART flight-control path, not telemetry. The bringup keeps it disabled and uses `betaflight_msp_controller` on a separate port instead.

## `msp_imu_driver_node`

Connects with `fcu::FlightController` from the `msp` library and subscribes to `RawImu` at `target_hz`, `Analog` at 10 Hz and `EscSensorData` at 20 Hz. Scaling: 512 LSB per g, gyro `raw / 4.096` deg/s. Publishes `/imu0`, `/battery` (voltage + current), `/motor_rpm`. Stamps with `now()`. No validation, no diagnostics.

## Files

| Path | Purpose |
| :--- | :--- |
| `src/perception/betaflight_blackbox_driver/src/blackbox_driver_node.cpp` | Blackbox decoder, validation, diagnostics, MSP control |
| `src/perception/betaflight_blackbox_driver/src/msp_imu_driver_node.cpp` | MSP poller |
| `scripts/perception/betaflight/setup_jetson_uart.sh` | Sets the UART clock to 80 MHz and `low_latency` |
| `scripts/perception/betaflight/record_imu.py`, `parse_blackbox.py`, `see_data.py` | Raw-capture helpers, independent of ROS |
| `docs/docs/files/betaflight_conf/*.txt` | Betaflight CLI dumps per drone, including the blackbox serial settings |

## Known limitations

- No hardware timestamp. Default stamps are the Jetson wall clock at publish time; `use_blackbox_time_for_stamp` uses the FC clock but the offset to ROS time is estimated from the first frame only.
- Publishing is decimation, not filtering: at `target_hz` 150 from a 2 kHz stream, 12 of 13 samples are dropped.
- The frame CRC depends on the custom firmware branch; against stock Betaflight every frame fails the CRC check.
- Build depends on the `msp` package, whose submodule entry is currently missing from the repository (see [Overview → Technical](../overview/technical.md#known-issues-repository-state-september-2026)).
