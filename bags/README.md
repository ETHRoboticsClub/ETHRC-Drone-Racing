# Bags

Convert a rosbag (Vicon + IMU) into the trajectory JSON schema.

## Layout

```
bags/
├── raw/          original recorded bags         (input,       gitignored)
├── filtered/     vicon+imu-only bags            (regenerable, gitignored)
├── processed/    *_traj.pkl, *_traj.json        (regenerable, gitignored)
└── tools/        scripts + configs              (tracked)
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
- `actions` is left empty (no command topics retained).
