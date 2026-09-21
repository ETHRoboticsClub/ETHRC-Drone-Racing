# IMU — workflow

**Owner:** [Ilyas Seckin](https://github.com/ilysec)

## Prerequisites

**Flight controller.** Flash the custom firmware with CRC-8 frames ([Betaflight → custom firmware](../../betaflight.md#building-custom-betaflight-firmware-with-crc-8-protection)), then in the Betaflight CLI:

```
feature BLACKBOX
set blackbox_device = SERIAL
set blackbox_mode = ALWAYS
set blackbox_sample_rate = 1/1
serial 0 128 1500000 57600 115200 1500000    # UART1 as blackbox at 1.5 Mbaud; adjust the port index
set blackbox_disable_pids = ON
set blackbox_disable_rc = ON
set blackbox_disable_setpoint = ON
set blackbox_disable_bat = OFF                # keep: voltage
set blackbox_disable_alt = ON
set blackbox_disable_rssi = ON
set blackbox_disable_debug = ON
set blackbox_disable_attitude = ON
set blackbox_disable_motors = OFF             # keep: motor outputs and eRPM
set blackbox_disable_rpm = OFF
save
```

Bidirectional DShot must be enabled in the Motors tab for eRPM. The per-drone CLI dumps under `docs/docs/files/betaflight_conf/` already contain these settings.

**Wiring.** FC UART TX → Jetson `/dev/ttyTHS1` RX, common ground. Pinouts: [Flight controller communication](../../drone-setup/drone_flight_controller_comunication.md).

**Jetson.** Once per boot, raise the UART clock and set low-latency mode (the node refuses to start at ≥ 1 Mbaud otherwise):

```bash
./scripts/perception/betaflight/setup_jetson_uart.sh
# equivalent to:
echo 80000000 | sudo tee /sys/kernel/debug/bpmp/debug/clk/uarti/rate
sudo setserial /dev/ttyTHS1 low_latency
```

Your user must be in `dialout`.

## Run

Through the bringup (default `imu_source:=blackbox`):

```bash
ros2 launch drone_control racing_bringup.launch.py msp_bench_mode:=true view_stream:=false \
  imu_port:=/dev/ttyTHS1 imu_baud:=1500000 target_hz:=150
```

The bringup passes `validate_imu_stream: false`. Standalone with validation on:

```bash
ros2 run betaflight_blackbox_driver blackbox_driver_node --ros-args \
  -p port:=/dev/ttyTHS1 -p baud:=1500000 -p target_hz:=150 -p validate_imu_stream:=true
```

MSP fallback (also from a laptop over USB):

```bash
ros2 launch drone_control racing_bringup.launch.py msp_bench_mode:=true imu_source:=msp \
  imu_port:=/dev/ttyUSB0 imu_baud:=115200
```

The blackbox stream only starts when Betaflight considers itself logging. With `blackbox_mode = ALWAYS` that is at boot; otherwise the node logs `Awaiting Dictionary (H)... Waiting for you to ARM the drone!` until you arm.

## Verify

```bash
ros2 topic hz /imu0                                        # ≈ target_hz
ros2 topic echo --once --qos-reliability best_effort /imu0  # z accel ≈ +9.8 at rest
ros2 topic echo --once --qos-reliability best_effort /motor_rpm
ros2 topic echo /blackbox_diagnostics --once
```

Every 10 s the node logs `Blackbox stream health: <kB/s>, decoded <fps> (I:n P:n), accepted <fps>, rejected n, resyncs n`. A healthy link shows ~2000 decoded fps, zero dropped bytes and rejected counts in the single digits over a session.

## Troubleshooting

| Symptom | Cause | Fix |
| :--- | :--- | :--- |
| `FATAL: UART Clock Rate is too low` and exit | `uarti` clock not raised | Run `setup_jetson_uart.sh` |
| `Failed to find serial port` | Wrong device or permissions | Check `ls -l /dev/ttyTHS1`, `dialout` membership |
| Bytes arrive, no frames, `Awaiting Dictionary` forever | Header never seen (node started mid-stream, or FC not logging) | Restart the FC or arm; `blackbox_mode = ALWAYS` |
| Every frame rejected with `CRC mismatch` | Stock Betaflight firmware | Flash the CRC branch |
| `eRPM values are all zero but motor outputs are non-zero` | Bidirectional DShot off | Enable it in the Motors tab |
| `eRPM fields NOT found in Blackbox header` | `blackbox_disable_rpm = ON` | Set it to `OFF` |
| Rejects in bursts, resyncs climbing | UART noise or wrong baud | `debug_raw:=true`, inspect `/tmp/capture.bbl`; check wiring and clock |
| Gyro/accel spikes reach consumers | Validation disabled (bringup default) | `validate_imu_stream:=true`, tune `max_*` limits only after confirming the spikes are not real motion |
| `Detected N publishers on /imu0` | Two IMU drivers running | Stop one (`imu_source` selects a single node) |

## Raw capture without ROS

```bash
python3 scripts/perception/betaflight/record_imu.py      # 10 s to blackbox.bin
python3 scripts/perception/betaflight/parse_blackbox.py blackbox.bin   # count I/P markers
```
