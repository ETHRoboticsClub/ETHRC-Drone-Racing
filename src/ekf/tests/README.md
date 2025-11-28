# EKF Scenario Test Procedure

The entire scenario workflow (mock publisher, Robot Localization EKF, bag recording, and CSV extraction) is automated by `ros2 launch ekf scenario_record.launch.py`. Use the manual steps below only if you need fine‑grained control.

---

## 1. Build the Package
```bash
colcon build --packages-select ekf
source install/setup.bash
```

## 2. Run a Scenario (Recommended Path)
Launch everything with a single command:
```bash
ros2 launch ekf scenario_record.launch.py \
  scenario:=const_vel \
  bagname:=defaultqp \
  duration:=30.0 \
  seed:=1 \
  vio_rate_hz:=100.0 \
  vision_rate_hz:=60.0 \
```

### What this launch does
- Starts `ekf/mock_publisher` with the provided parameters.
- Includes `rl_ekf.launch.xml` (Robot Localization) without modification.
- Records `/vio/odometry`, `/vision/pose`, `/odometry/filtered`, `/tf`, and `/tf_static` via `ros2 bag`.
- Mirrors `ros2 topic echo /ekf/odometry --csv` and writes the stream to `scenario_data.csv` with the canonical header line you specified.
- Shuts everything down cleanly when the mock publisher’s duration elapses.

### Output layout
- Bags are stored under `tests/<scenario>_<bagname>/`.
- Example: with `bagname:=defaultqp` and `scenario:=const_vel`, a run on 2025‑11‑11 at 19:05:02 creates `tests/const_vel_defaultqp/`.
- `scenario_record.launch.py` pre-creates the scenario directory (for CSV logging) and tells `ros2 bag record` to write into the `rosbag/` subdirectory to avoid conflicts.
- Inside that directory you will find:
  - `rosbag/metadata.yaml` and the recorded `rosbag/rosbag2_*` files.
  - `scenario_data.csv` containing `/ekf/odometry` samples; the first line is  
    ```
    t_sec,t_nsec,_,_,x,y,z,xq,yq,zq,w,pose_cov_0,pose_cov_1,pose_cov_2,pose_cov_3,pose_cov_4,pose_cov_5,pose_cov_6,pose_cov_7,pose_cov_8,pose_cov_9,pose_cov_10,pose_cov_11,pose_cov_12,pose_cov_13,pose_cov_14,pose_cov_15,pose_cov_16,pose_cov_17,pose_cov_18,pose_cov_19,pose_cov_20,pose_cov_21,pose_cov_22,pose_cov_23,pose_cov_24,pose_cov_25,pose_cov_26,pose_cov_27,pose_cov_28,pose_cov_29,pose_cov_30,pose_cov_31,pose_cov_32,pose_cov_33,pose_cov_34,pose_cov_35,vx,vy,vz,wx,wy,wz,twist_cov_0,twist_cov_1,twist_cov_2,twist_cov_3,twist_cov_4,twist_cov_5,twist_cov_6,twist_cov_7,twist_cov_8,twist_cov_9,twist_cov_10,twist_cov_11,twist_cov_12,twist_cov_13,twist_cov_14,twist_cov_15,twist_cov_16,twist_cov_17,twist_cov_18,twist_cov_19,twist_cov_20,twist_cov_21,twist_cov_22,twist_cov_23,twist_cov_24,twist_cov_25,twist_cov_26,twist_cov_27,twist_cov_28,twist_cov_29,twist_cov_30,twist_cov_31,twist_cov_32,twist_cov_33,twist_cov_34,twist_cov_35
    ```

### Fine-tuning the EKF
To fine-tune the EKF go into config/rl_ekf.yaml. The main tweaking parameters are:
- process_noise_covariance
- initial_estimate_covariance

### Helpful follow-up commands
```bash
# Summaries
ros2 bag info tests/<scenario>_<bagname>/rosbag
head -n 5 tests/<scenario>_<bagname>/scenario_data.csv

# Replay to visualize topics
ros2 bag play tests/<scenario>_<bagname>/rosbag
```

### Scenario catalog (`mock_publisher.cpp`)
All scenarios stream to `/vio/odometry` and `/vision/pose` simultaneously. VIO samples use the higher‑noise AGI settings  
(position σ = 1e‑2 m, attitude σ = 1e‑3 rad, linear velocity σ = 1e‑2 m/s, angular velocity σ = 1e‑1 rad/s), while the vision pose reports the same nominal trajectory with tighter noise (position σ = 2e‑3 m, attitude σ = 1e‑3 rad). Change `seed` to repeatably sweep the Gaussian noise.

#### `rest` — stationary sanity check
- Trajectory: `p(t) = (0, 0, 0)` and zero linear/angular velocity; only sensor noise perturbs the readings.
- Orientation: identity quaternion plus 1e‑3 rad noise per axis for both VIO and vision.
- Launch:
  ```bash
  ros2 launch ekf scenario_record.launch.py \
    scenario:=rest \
    bagname:=rest_baseline \
    duration:=20.0
  ```

#### `const_vel` — 0.5 m/s straight line
- Trajectory: `x(t) = 0.5 t`, `y(t) = 0`, `z(t) = 0`. Linear velocity is exactly `(0.5, 0, 0)` m/s; VIO angular velocity is zero.
- Orientation: identity quaternion with 1e‑3 attitude noise; no commanded yaw.
- Launch:
  ```bash
  ros2 launch ekf scenario_record.launch.py \
    scenario:=const_vel \
    bagname:=const_vel_defaultqp \
    duration:=30.0
  ```

#### `turn` — on-the-spot yaw spin
- Trajectory: the vehicle stays at the origin (`p(t)=0`) while yawing at `ω = 0.5` rad/s. Quaternion: `z = sin(0.25 t)`, `w = cos(0.25 t)` plus noise; roll/pitch remain zero.
- Twist: angular velocity `(0, 0, 0.5)` rad/s, linear velocity remains zero aside from Gaussian noise.
- Launch:
  ```bash
  ros2 launch ekf scenario_record.launch.py \
    scenario:=turn \
    bagname:=turn_in_place \
    duration:=20.0
  ```

#### `accel` — constant +X acceleration
- Trajectory: `x(t) = 0.5 * 0.5 * t^2 = 0.25 t^2` m, `y(t)=0`, `z(t)=0`. Velocity grows linearly: `v_x(t) = 0.5 t` m/s, `v_y = v_z = 0`.
- Orientation: identity quaternion with 1e‑3 noise, matching both sensors; angular velocity is zero.
- Launch:
  ```bash
  ros2 launch ekf scenario_record.launch.py \
    scenario:=accel \
    bagname:=accel_x_0p5 \
    duration:=20.0
  ```

---

## 3. Manual / Legacy Workflow (Optional)
If you prefer to manage each process yourself, follow the steps below. They mirror the automated launch but require multiple terminals.

1. **Mock Publisher**
   ```bash
   ros2 run ekf mock_publisher
   ```
2. **Robot Localization EKF**
   ```bash
   ros2 launch ekf rl_ekf.launch.xml
   ```
3. **Rosbag Recording**
   ```bash
   ros2 bag record -o tests/<bagname>_<scenario>_<timestamp> \
     /ekf/odometry /vio/odometry /vision/pose /tf /tf_static
   ```
4. **CSV Extraction**
   ```bash
   ros2 bag play tests/<bagname>_<scenario>_<timestamp>
   ros2 topic echo /ekf/odometry --csv \
     > tests/<bagname>_<scenario>_<timestamp>/scenario_data.csv
   ```
   Prepend the header shown above if you capture data this way—`scenario_record.launch.py` already does it for you.

---

## 4. Tips
- Choose descriptive `bagname` values (e.g., `defaultqp`) to keep the generated directories tidy.
- Tune `scenario`, `duration`, `seed`, `vio_rate_hz`, `vision_rate_hz`, or `rl_ekf_path` via CLI arguments.
- The `tests/` directory is safe to commit; large bag artifacts should be cleaned or git-ignored if necessary.
