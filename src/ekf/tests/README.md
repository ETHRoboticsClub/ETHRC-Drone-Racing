# EKF Test Procedure

## 1. Build
Open a new terminal and run:
```bash
colcon build
```
Then source the workspace:
```bash
source install/setup.bash
```

## 2. Start Mock Data Publisher
In the same (built) terminal:
```bash
ros2 run ekf mock_publisher
```

## 3. Start Robot Localization (EKF)
Open a second terminal, source the workspace again, then launch:
```bash
source install/setup.bash
ros2 launch ekf rl_ekf.launch.xml
```

## 4. Record a ros2 bag
Open a third terminal, source the workspace, then record. Use clear naming!

Record for ~60–120 s:
```bash
source install/setup.bash
ros2 bag record -o <your-super-creative-bag-name> /ekf/odometry /vio/odometry /vision/pose /tf /tf_static
```
Stop with Ctrl+C after sufficient duration.

## 5. Convert Topic Output to CSV (Post-Processing)
Replay the bag (can be a new terminal):
```bash
source install/local_setup.bash
ros2 bag play tests/<your-super-creative-bag-name>
```

While it plays, echo the topic to CSV:
```bash
ros2 topic echo /ekf/odometry --csv > tests/<your-super-creative-bag-name>/<your-super-creative-bag-name>.csv
```

## 6. Post-processing
Add this to the top-line of the csv:
```bash
t_sec,t_nsec,_,_,x,y,z,xq,yq,zq,w,pose_cov_0,pose_cov_1,pose_cov_2,pose_cov_3,pose_cov_4,pose_cov_5,pose_cov_6,pose_cov_7,pose_cov_8,pose_cov_9,pose_cov_10,pose_cov_11,pose_cov_12,pose_cov_13,pose_cov_14,pose_cov_15,pose_cov_16,pose_cov_17,pose_cov_18,pose_cov_19,pose_cov_20,pose_cov_21,pose_cov_22,pose_cov_23,pose_cov_24,pose_cov_25,pose_cov_26,pose_cov_27,pose_cov_28,pose_cov_29,pose_cov_30,pose_cov_31,pose_cov_32,pose_cov_33,pose_cov_34,pose_cov_35,vx,vy,vz,wx,wy,wz,twist_cov_0,twist_cov_1,twist_cov_2,twist_cov_3,twist_cov_4,twist_cov_5,twist_cov_6,twist_cov_7,twist_cov_8,twist_cov_9,twist_cov_10,twist_cov_11,twist_cov_12,twist_cov_13,twist_cov_14,twist_cov_15,twist_cov_16,twist_cov_17,twist_cov_18,twist_cov_19,twist_cov_20,twist_cov_21,twist_cov_22,twist_cov_23,twist_cov_24,twist_cov_25,twist_cov_26,twist_cov_27,twist_cov_28,twist_cov_29,twist_cov_30,twist_cov_31,twist_cov_32,twist_cov_33,twist_cov_34,twist_cov_35
```