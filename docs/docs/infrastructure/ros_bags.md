# ROS Bags & Flight Data (S3)

**Page Owner:** [Aeronautics Division](https://github.com/orgs/ETHRoboticsClub/teams/aeronautics/)

---

This page serves as a central registry for flight data uploaded to our AWS S3 storage. These bags are essential for offline tuning of perception algorithms, MPC modeling, and RL training.

## 🗃️ S3 Data Registry

| Date | Person | Drone | S3 Location (s3://...) | Notes | Useful Topics |
| :--- | :--- | :--- | :--- | :--- | :--- |
| 2026-05-17 | zephyrus | zephyrus | `s3://ethrc-aeronautics/droneracing/bags/2026-05-17-zephyrus-imu-cam-calib5.zip` | imu-cam-calib5 | `/battery`, `/cam0/camera_info`, `/cam0/image_h264`, `/cam0/image_rgb`, `/drone_attitude_cmd`, `/drone_current_target`, `/drone_position_cmd`, `/drone_state_sim`, `/imu0`, `/motor_rpm`, `/tf`, `/tf_static` |
| 2026-05-17 | zephyrus | zephyrus | `s3://ethrc-aeronautics/droneracing/bags/2026-05-17-zephyrus-multi-gate-rgb-only1.zip` | multi_gate_rgb_only1 | `/battery`, `/cam0/camera_info`, `/cam0/image_h264`, `/cam0/image_rgb`, `/drone_attitude_cmd`, `/drone_current_target`, `/drone_position_cmd`, `/drone_state_sim`, `/imu0`, `/motor_rpm`, `/vicon/*` |
| 2026-05-17 | zephyrus | zephyrus | `s3://ethrc-aeronautics/droneracing/bags/2026-05-17-zephyrus-multi-gate2.zip` | multi_gate2, low RGB framerate because raw `/cam0/image_rgb` was recorded; prefer H.264 for color video | `/cam0/camera_info`, `/cam0/image_h264`, `/cam0/image_rgb`, `/imu0`, `/motor_rpm`, `/vicon/*` |
| 2026-05-12 | Dimetri | Zephyrus | `s3://ethrc-aeronautics/droneracing/bags/2026-05-12_dimetri_zephyrus-calib4-rgb.zip` | AprilTag calibration pattern for cam-imu calibration using RGB images. | `/cam0/image_raw`, `/imu0` |
| 2026-05-09 | Dimetri | Zephyrus | `s3://ethrc-aeronautics/droneracing/bags/zephyrus-calib3-imu-cam.zip` | Calibration run for IMU-camera calibration. Monochrome image only. | `/cam0/image_raw`, `/imu0` |
| 2026-05-06 | Dimetri | Zephyrus | `s3://ethrc-aeronautics/droneracing/bags/06-05-26-zephyrus-calib2.zip` | Calibration run with the old h264 compressed video feed. Shouldn't be used anymore | `/image_raw/h264`, `/imu0` |
| *Template* | *Name* | *Drone* | `s3://ethrc-drone-racing/bags/...` | *Short description* | *topics* |

---

## 📥 How to Access Data

### 1. Prerequisites
Ensure you have the AWS CLI installed and configured with your credentials. You either need your team's key or the machine key. Both are shared internally and need to be requested. See [AWS Account Setup](aws_setup.md) for details.

### 2. Downloading a Bag
Use the `aws s3 cp` command with the `--recursive` flag to download a bag directory:

```bash
# Example download
aws s3 cp s3://ethrc-aeronautics/droneracing/YOUR_BAG_NAME/ ./local_directory/ --recursive
```

### 3. Listing Available Bags
To see what else is in the bucket:
```bash
aws s3 ls s3://ethrc-aeronautics/droneracing/bags/
```

---

## 📤 Contributing Data

When uploading new bags, please follow these conventions:
1.  **Naming**: Use the format `YYYY-MM-DD-DRONE_NAME-DESCRIPTION`.
2.  **Organization**: Upload to the `bags/` prefix.
3.  **Documentation**: Update this page with a new entry in the registry table.
4.  **Filtering**: If possible, remove high-bandwidth topics that aren't needed for the specific analysis to save storage costs.

### Interactive Recording Helper

Use the TUI helper on the drone after sourcing ROS and the workspace:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
./scripts/tools/record_rosbag_tui.py
```

The helper selects all live non-NITROS topics by default, including
`/cam0/image_h264` for color video. It leaves `.zip` creation disabled by
default; press `z` to zip after recording, or `u` to zip and upload. You can
still choose common presets such as IMU-camera calibration, name the bag using
the `YYYY-MM-DD-DRONE_NAME-DESCRIPTION` scheme, and optionally upload the zip to
`s3://ethrc-aeronautics/droneracing/bags/`. When it finishes, it prints a draft
registry row that can be pasted into the table above.

### Racing Bench-Mode Recording

For `msp_bench_mode` tests, prefer the helper defaults or manually record
compressed color video plus telemetry:

```bash
ros2 bag record \
  /cam0/image_h264 \
  /cam0/camera_info \
  /imu0 \
  /motor_rpm \
  /vicon/drone/drone
```

Avoid recording `/cam0/image_rgb` during normal flight or bench runs. It is a
large raw RGB debug/gate-detection topic and can record at a much lower
effective FPS than the camera. Use `/cam0/image_h264` for color video and
`/cam0/image_raw` only for short mono debug or calibration captures.
