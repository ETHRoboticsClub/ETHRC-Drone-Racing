# ETHRC-Drone-Racing

This repo gives us a ready-to-use drone racing environment in Docker.
Why Docker? → So everyone works in the same environment, avoiding “it works on my machine” issues.

It bundles:

ROS 2 Humble → our robotics framework

Gazebo (gz) → 3D simulator for drones

PX4 SITL → autopilot software in simulation

MAVROS / MAVSDK → talk between PX4 and ROS

Workspace /ws → where we put our ROS 2 packages

> **Host OS:** Ubuntu/Linux recommended  
> **Requires:** Docker Engine + Compose v2

---

## Quick Start

### Step 0 — One-time: Set up SSH for GitHub (if you’ve never done this)
We need SSH so git clone works smoothly without typing passwords.
Follow GitHub’s guide: [Follow Github's guide here](https://docs.github.com/en/authentication/connecting-to-github-with-ssh)

### Step 1 — Clone (with PX4 submodule)
```bash
git clone --recurse-submodules git@github.com:ETHRoboticsClub/ETHRC-Drone-Racing.git
cd ETHRC-Drone-Racing
git checkout perception_ArUco_Gate_Estimation
git pull
git submodule update --init --recursive
```

### Step 2 — Build the Docker image
```bash
docker compose build
```

### Step 3 — Start services (enable GUI, start container, open shell)
```bash
# Allow container GUI apps to open windows on your desktop (X11)
xhost +local:root
cd px4/PX4-Autopilot
sudo rm -rf build
sudo chown -R $USER:$USER .
cd ../..
# Start the container (detached) and open a shell inside
docker compose up -d
docker exec -it erc-aeronautics bash
```
# DEBUG
```bash
git config --global --add safe.directory '*'
```
```bash
sudo chown -R user:user /your_path_to_file/ETHRC-Drone-Racing/.colcon/
```
replace user and path

### Run the simulation in the container
```bash
# Default x500 in the default world
source install/setup.bash
cd /ws/px4/PX4-Autopilot
make clean
make px4_sitl gz_x500_cam 
```

### Run the Ground truth
```bash
docker exec -it erc-aeronautics bash
source install/setup.bash
ros2 run pointcloud_gt pointcloud_GT

```

### Bridge the camera topic 
```bash
docker exec -it erc-aeronautics bash
source install/setup.bash
ros2 launch camera_plugin camera_bridge.launch.py
```
Creates:
```bash
Gazebo  →  ROS 2
/drone/camera/image
/drone/camera/camera_info
/model/x500_cam_0/odometry
```

### View the camera feed 
In a new container:
```bash
docker exec -it erc-aeronautics bash
source install/setup.bash
ros2 run image_viewer_pkg image_viewer
```


### Run detedction Node with parameters
```bash
docker exec -it erc-aeronautics bash
source install/setup.bash
ros2 launch aruco_pose_estimation aruco_pose_estimation.launch.py
```

### Scripts
## Hover at position (1,1,1) facing origin
 ```bash
 docker exec -it erc-aeronautics bash
 python3 ./src/scripts/hover_at_position.py 
 ```
## Orbit facing origin
```bash
docker exec -it erc-aeronautics bash
python3 ./src/scripts/orbit_around_origin.py 
```


#### Launch RViz2 Without Configuration (Manual Setup)
```bash
rviz2
```
Then manually add displays and configure topics.

## Adding Custom Worlds

Place worlds here:
```bash
px4/PX4-Autopilot/Tools/simulation/gz/worlds/
```

Set which world is loaded in:
```bash
px4/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/4022_gz_x500_cam
PX4_GZ_WORLD=<world_name>
```

### RECORD ROS BAGS
```bash
ros2 bag record -o ~/ros_ws/src/rosbags/benchmark_$(date +%Y%m%d_%H%M%S) /aruco/drone_pose /aruco/drone_pose /model/x500_cam_0/odometry
```

## Command the drone to fly
Use Mavsdk API (offboard or action library) or Qgroundcontrol 