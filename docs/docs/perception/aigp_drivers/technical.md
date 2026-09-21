# AIGP simulator drivers — technical

**Owner:** [Jules Schmutz](https://github.com/juliomathis)

The AI Grand Prix (AIGP) Virtual Qualifier ships a Windows simulator (`FlightSim.exe`) that streams its camera as chunked JPEG over UDP and its IMU as MAVLink `HIGHRES_IMU`. Two C++ components translate those streams into the same `/cam0/*` and `/imu0` topics the real drone produces, so the rest of the stack does not know it is in a simulator.

| Package | Node (component) | Input | Output |
| :--- | :--- | :--- | :--- |
| `aigp_camera_driver` | `aigp_camera_driver::VisionDriverNode` | UDP `:5600`, chunked JPEG | `/cam0/image_raw` (mono8), `/cam0/image_rgb` (rgb8), `/cam0/camera_info` |
| `aigp_imu_driver` | `aigp_imu_driver::ImuBridgeNode` | UDP `:14550`, MAVLink `HIGHRES_IMU` (id 105) | `/imu0` |

Both are composable nodes with intra-process communication and best-effort (`SensorDataQoS`) publishers. The wider AIGP stack (bridge, state estimator, policy, colour gate detector) is documented under [AIGP Simulation](../../controls/aigp_simulation.md).

## Camera driver

Wire format (spec VADR-TS-002 §4.6): each datagram is a 24-byte little-endian header followed by a JPEG slice.

| Field | Type | Meaning |
| :--- | :--- | :--- |
| `frame_id` | `uint32` | Frame sequence number |
| `chunk_id`, `total_chunks` | `uint16` | Slice index and count |
| `jpeg_size`, `payload_size` | `uint32` | Full JPEG size, this slice's size |
| `sim_time_ns` | `uint64` | Simulator timestamp |

Behaviour:

- Slices are reassembled per `frame_id`; a frame is complete when `total_chunks` slices arrived. Frames older than the newest one by more than one id are evicted. Frame ids must increase: a completed id is never rebuilt (the simulator sends each frame twice), and a simulator restart that resets ids stalls the node until it is restarted.
- Decoding is lazy: the JPEG is decoded only if `image_raw` or `image_rgb` has a subscriber, and `camera_info` is published only when it has one too.
- `camera_info` is the spec's pinhole: 640×360, `fx = fy = 320`, `cx = 320`, `cy = 180`, no distortion (`plumb_bob`, zeros). The camera sits at the body origin tilted 20° up; that extrinsic belongs in the OpenVINS config, not here.
- Stamps: `use_sim_timestamp: false` (default) → local receipt time; `true` → `sim_time_ns`.

Parameters: `udp_port` (5600), `udp_bind_address` (`0.0.0.0`), `width`/`height` (640/360), `frame_id` (`cam0`), `use_sim_timestamp`, `camera_matrix`, `distortion_coefficients`, `distortion_model`. Receive buffer 16 MiB.

## IMU driver

Parses the raw MAVLink byte stream with the C library (the simulator identifies as `autopilot=GENERIC`, so MAVSDK would not attach) and republishes every `HIGHRES_IMU`: body-FRD accelerometer (m/s²) and gyroscope (rad/s) copied as-is, `frame_id: imu_link`, `orientation_covariance[0] = -1` (no orientation).

Parameters: `connection_url` (`udpin://0.0.0.0:14550`; only the port is used, the bind address is always any), `imu_topic` (`/imu0`), `frame_id`, `use_sim_timestamp` (**default `true`** → stamps from `time_usec`, simulator boot-relative).

## Timestamp caveat

The simulator's camera stamps are server epoch time and its IMU stamps are boot-relative, so the two cannot be mixed. With the shipped defaults the camera driver stamps at receipt and the IMU driver stamps with simulator time — two different clocks on the topics OpenVINS fuses. Set `use_sim_timestamp:=false` on the IMU driver (or `true` on both and convert) before running VIO in the simulator.

`aigp_bringup.launch.py` sidesteps this: it starts the camera driver, but takes `/imu0` from `aigp_bridge_node` (`src/utils/betaflight_mavlink_bridge`), which also listens on 14550 and stamps at receipt. Do not run `aigp_imu_driver` alongside the bridge: both bind UDP 14550 and each datagram reaches only one of them.

## Files

| Path | Purpose |
| :--- | :--- |
| `src/perception/aigp_camera_driver/src/vision_driver_node.cpp` | Camera component |
| `src/perception/aigp_camera_driver/config/camera_params.yaml`, `launch/aigp_camera.launch.py` | Defaults and container launch |
| `src/perception/aigp_imu_driver/src/imu_bridge_node.cpp` | IMU component |
| `src/perception/aigp_imu_driver/config/imu_params.yaml`, `launch/aigp_imu.launch.py` | Defaults and container launch |
| `src/drone_control/launch/aigp_bringup.launch.py` | Full simulator bringup (uses the camera driver, not the IMU driver) |
| `src/perception/gate_detection/gate_detection/detector_node.py` | Colour gate detector for the simulator ([Gate detection → Technical](../gate_detection/technical.md#aigp-colour-detector-detector_nodepy)) |

## Known limitations

- Frame-id monotonicity: restart the camera driver after restarting the simulator.
- No handling of the simulator's TIMESYNC; clocks are as described above.
- The IMU driver ignores `udp_bind_address`.
