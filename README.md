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
```

### Step 2 — Build the Docker image
```bash
docker compose build
```

### Step 3 — Start services (enable GUI, start container, open shell)
```bash
# Allow container GUI apps to open windows on your desktop (X11)
xhost +local:root

# Start the container (detached) and open a shell inside
docker compose up -d
docker exec -it erc-aeronautics bash
```

### Step 4 — Run the simulation in the container
```bash
# Default x500 in the default world
cd /ws/px4/PX4-Autopilot
make px4_sitl gz_x500
```

# Using simulated camera

## X500 Cam Simulation (PX4 + Gazebo + ROS 2)

### Step 1 — Pull the changes with submodules
```bash
git pull
git submodule update --init --recursive
```
### Step 2 — Fix PX4 permissions (host machine), if you have any
```bash
cd px4/PX4-Autopilot
sudo rm -rf build
sudo chown -R $USER:$USER .
```
```bash
git config --global --add safe.directory '*'
```
```bash
sudo chown -R user:user /your_path_to_file/ETHRC-Drone-Racing/.colcon/
```
replace user and the path

### Step 3 — Enter the simulation container
```bash
docker exec -it erc-aeronautics bash
```

### Step 4 — Run the simulation (inside container)
```bash
cd /workspaces/ETHRC-Drone-Racing/px4/PX4-Autopilot
make clean
make px4_sitl gz_x500_cam
```

This launches PX4 + Gazebo Harmonic + the x500_cam model.

### Step 5 — Build ROS workspace 
In a new container:
```bash
cd ~/ros_ws
colcon build
source install/setup.bash
```

### Step 6 — Bridge the camera topic 
```bash
ros2 launch camera_plugin camera_bridge.launch.py
```
Creates:
```bash
Gazebo  →  ROS 2
/drone/camera/image
```
If you do ros2 topic list in another terminal, you should see the topic /drone/camera/image
### Step 7 — View the camera feed 
In a new container:
```bash
cd ~/ros_ws
source install/setup.bash
ros2 run image_viewer_pkg image_viewer
```


### Run detedction Node with parameters
```bash
ros2 launch aruco_pose_estimation aruco_pose_estimation.launch.py     camera_topic:=/drone/camera/image     camera_calibration_file:=/home/jules/ros_ws/src/aruco_pose_estimation/config/camera_calibration_simulation.yaml     marker_map_file:=/home/jules/ros_ws/src/aruco_pose_estimation/config/marker_map.yaml     world_frame:=world     camera_frame:=camera_link     publish_tf:=true     publish_debug_image:=true
```

## Script
 to hover at (1,1,1) adn (0,0,-3.14)

 ```bash
 python3 ./src/scripts/hover_at_position.py 
 ```

### RVIZ2

 ```bash
rviz2 
```
Add topic you need visualized

(TODO:predefined rviz setup)

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

## Command the drone to fly
Use Mavsdk API (offboard or action library) or Qgroundcontrol 