# ROS Bags & Flight Data (S3)

**Page Owner:** [Aeronautics Division](https://github.com/orgs/ETHRoboticsClub/teams/aeronautics/)

---

This page serves as a central registry for flight data uploaded to our AWS S3 storage. These bags are essential for offline tuning of perception algorithms, MPC modeling, and RL training.

## 🗃️ S3 Data Registry

| Date | Person | Drone | S3 Location (s3://...) | Notes | Useful Topics |
| :--- | :--- | :--- | :--- | :--- | :--- |
| 2026-05-09 | Dimetri | Zephyrus | `s3://ethrc-aeronautics/droneracing/bags/zephyrus-calib3-imu-cam.zip` | Calibration run with the NITROS cam stack for imu-cam calibration. Monochrome image only. | `/cam0/image_raw`, `/imu0` |
| 2026-05-06 | Dimetri | Zephyrus | `s3://ethrc-aeronautics/droneracing/bags/06-05-26-zephyrus-calib2.zip` | Calibration run with the old h264 compressed video feed. Shouldn't be used anymore | `/image_raw/h264`, `/imu0` |
| *Template* | *Name* | *Drone* | `s3://ethrc-drone-racing/bags/...` | *Short description* | *topics* |

---

## 📥 How to Access Data

### 1. Prerequisites
Ensure you have the AWS CLI installed and configured with your credentials. You either need your teams key or the maschine key. Both are shared internally and need to be requested.See [AWS Account Setup](aws_setup.md) for details.

### 2. Downloading a Bag
Use the `aws s3 cp` command with the `--recursive` flag to download a bag directory:

```bash
# Example download
aws s3 cp s3://ethrc-drone-racing/bags/YOUR_BAG_NAME/ ./local_directory/ --recursive
```

### 3. Listing Available Bags
To see what else is in the bucket:
```bash
aws s3 ls s3://ethrc-drone-racing/bags/
```

---

## 📤 Contributing Data

When uploading new bags, please follow these conventions:
1.  **Naming**: Use the format `YYYY-MM-DD-DRONE_NAME-DESCRIPTION`.
2.  **Organization**: Upload to the `bags/` prefix.
3.  **Documentation**: Update this page with a new entry in the registry table.
4.  **Filtering**: If possible, remove high-bandwidth topics that aren't needed for the specific analysis to save storage costs.