# Bags

Convert a rosbag (Vicon + IMU + joystick) into the trajectory JSON schema.

## Layout

```
bags/
├── raw/          original recorded bags             (input,       gitignored)
├── filtered/     vicon+imu+joystick-only bags       (regenerable, gitignored)
├── processed/    *_traj.pkl, *_traj.json            (regenerable, gitignored)
└── tools/        scripts + configs                  (tracked)
```

Only `tools/` and this README are checked into git. `raw/`, `filtered/`, and
`processed/` are created automatically by `bag_to_json.sh`.

## One-shot

```bash
tools/bag_to_json.sh raw/<bag_name>
# writes filtered/<bag_name>/, processed/<bag_name>_traj.{pkl,json}
```

Pass a path to the **bag directory** (containing `metadata.yaml`), not a `.db3` file.

## Manual steps

```bash
source /opt/ros/jazzy/setup.bash

python3 tools/preprocess_bag.py raw/<bag_name> \
    --out-bag filtered/<bag_name> \
    --out-pkl processed/<bag_name>_traj.pkl \
    [--rate 100] [--skip-filter]

python3 tools/to_json.py processed/<bag_name>_traj.pkl \
                        processed/<bag_name>_traj.json
```

## Notes

- `linear_velocity` is finite-differenced from Vicon position with EMA smoothing
  (`v = 0.5*v_prev + 0.5*(p_cur - p_prev)/dt`), mirroring `state_bridge_node.py`.
- `angular_velocity` comes directly from `/imu0`.
- Quaternion order: **xyzw**.
- `actions` is populated from `/joystick_inputs` (Betaflight `rcCommand`,
  published by `betaflight_blackbox_driver`). Each sample is converted to a
  physical IsaacLab action `[roll_rate, pitch_rate, yaw_rate, thrust]`
  (rad/s, rad/s, rad/s, N) by `tools/from_betaflight.py`. If the topic was not
  recorded, `actions` is left empty.

### Joystick → action conversion details

- `/joystick_inputs` is a headerless `std_msgs/Float64MultiArray`, so it is
  timestamped by **bag receive time**, a different clock from the Vicon/IMU
  header stamps that define the trajectory timeline. The two streams do **not**
  start together. They are aligned using `/imu0` as a shared reference: the IMU
  and joystick are both published by `betaflight_blackbox_driver`, so the IMU
  carries both clocks per message, and the median `header - bag` offset maps
  joystick bag-time onto the trajectory clock (see `align_joystick_time`). The
  aligned stream is then resampled with a **zero-order hold** (commands held
  until the next sample, not interpolated).
- If the joystick stream does not span the whole trajectory (e.g. teleop
  started late or stopped early), `preprocess_bag.py` prints a **coverage
  warning**; grid samples outside the joystick range are end-clamped (held),
  not real pilot input — trim those rows before training on them.
- Stick → body rate uses the Betaflight rate curve (`rcRate`, `superRate`,
  `rcExpo`). Override per-FC values with `--rc-rate/--super-rate/--rc-expo` on
  `to_json.py`.
- **Frame:** Betaflight is FRD, the sim env is FLU. Pitch and yaw are negated
  by default (matching `compute_actions()` in `src/rl/rl/hovering_experiment.py`,
  which negates pitch/yaw going sim→FC). Disable with `--no-invert-pitch` /
  `--no-invert-yaw`.
- **Thrust:** linear PWM 1000–2000 → `[0, thrust_max_n]` N (`--thrust-max-n`,
  default 34.192). Verify this matches your env's expected thrust units.
- Standalone (without a bag), convert a Betaflight rcCommand JSON directly:
  `python3 tools/from_betaflight.py --input log.json --output run.json --invert-pitch --invert-yaw`.
