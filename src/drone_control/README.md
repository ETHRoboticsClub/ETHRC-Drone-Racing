# Setup

Clone this branch recursively and build+run the docker container. When rebuilding, delete the old image and clear the cache (```docker system prune```).
```bash
git clone --recurse-submodules git@github.com:ETHRoboticsClub/ETHRC-Drone-Racing.git
cd ETHRC-Drone-Racing
# Prepare host volumes
./scripts/prepare_colcon_volumes.sh

# Start the development container
xhost +local:root
docker compose up mpc -d
docker compose exec mpc bash

Open 4 terminals and enter the container in each one.
```bash
docker exec -it erc-aeronautics bash
```

### Simulation vs Hardware deployment
Modify the [config](config/drone_controller_config.yaml) file to use the correct topic and message type for state updates, depending if we are on simulation or real hardware: 

## Terminal 1
Run gazebo
```bash
cd ~/ros_ws/px4/PX4-Autopilot && make px4_sitl gz_x500
```
## Terminal 2
Build ROS, then launch the drone controller
```bash
colcon build --packages-select drone_control
source install/setup.bash
ros2 launch drone_control drone_controller.launch.py
```

If you only changed `drone_control`, this rebuild is sufficient. You can also use symlink install for faster iteration:
```bash
colcon build --packages-select drone_control --symlink-install
source install/setup.bash
```

## Terminal 3
Send manual pose goals after setting `waypoints_file: ""` in `src/drone_control/config/drone_controller_config.yaml`.
This disables the default trajectory so the drone will wait for explicit target poses.
```bash
source install/setup.bash
ros2 run drone_control send_pose_goal
```
Then follow the interactive prompt to enter `x`, `y`, `z`, and `yaw` for each new target.

If you want to run a full trajectory goal sequence instead, use:
```bash
ros2 run drone_control send_trajectory_goal goal_only
```

## Terminal 4
If you wish to cancel a goal: 
```bash
source install/setup.bash
ros2 run drone_control send_pose_goal cancel
```
This might cause bugs and it is therefore recommended to wait out goals instead of cancelling them unnecessarily.

## Finally
Now that the first goal has been set, in the PlotJuggler at "Layout", load ```src/drone_control/config/control_plotjuggler.xml```

## Remarks
- The drone starts from the ground. It is therefore necessary to ensure the first goal does not contain changes in the x and y coordinates which could lead to it leaning into the ground immediately.
- The drone sometimes disarms itself automatically. If a pose goal is rejected by the server or the drone seems to not react, reactivate it with ```ros2 service call /arm_drone std_srvs/srv/SetBool "{data: true}"```


# Run the node
This is part of the old README. Expect some parts to be redundant with the "Setup" section above.

This runs the mpc controller && the plotjuggler visualization
```sh
ros2 launch drone_control drone_controller.launch.py
```

### Launch Arguments
| Argument | Default | Description |
|----------|---------|-------------|
| `plotjuggler` | `false` | Launch PlotJuggler visualization alongside the controller |

Example with PlotJuggler:
```sh
ros2 launch drone_control drone_controller.launch.py plotjuggler:=true
```

You need to run the simulation first:
```sh
cd ~/ros_ws/px4/PX4-Autopilot && make px4_sitl gz_x500_cam
```

You can test the action client with (this is a placeholder node, since this functionalities will be implemented using GUI):

Position goals
```sh
ros2 run drone_control send_pose_goal goal_only -x 0.0 -y 0.0 -z -1.0 -yaw 0.0
```

Trajectory goals:
1. Modify the [yaml_file](src/drone_control/config/control_plotjuggler.xml)
2. Run
```sh
ros2 run drone_control send_trajectory_goal goal_only
```
04/03/2026 Temporary Update:
To run the simulation toy need to:

First terminal:
```sh
cd ~/ros_ws/px4/PX4-Autopilot && make px4_sitl gz_x500
```
Second terminal:
```sh
ros2 run drone_control drone_controller_node
```

Third terminal:
```sh
ros2 run my_mavsdk mavsdk_client
```

Fourth terminal:
```sh
ros2 run drone_control send_pose_goal goal_only -x 0.0 -y 0.0 -z -1.0 -yaw 0.0
```
# Integration with GS and Perception - Seperate Package RoadMap:
> flags: MWP: minimum working product/ done 

1. MavSDK node **MWP**
    - state in ros_topic **done**
    - controls sub **done**
    - Connecting **done**
    - Actions for landing-taking off && arm/disarm **done**
    - Imu and other data (e.g. battery specifically) **WIP**
    - Debugging
2. Controller node  **done** 
    - Publish Controls **done**
    - Subscribe to state (Perception and Sim) **done**
    - Trajectory && Goal actions **done**
    - Debugging
3. GS package
    - Subscribe to odometry (sim/perception)
    - Subscribe to Controls input
    - Debugging
    - Trigger for position goals
    - Trigger for trajectory goals
4. Perception
    - Publish state
    - Subscribe to controls input
    - Debugging
    - other sim


# TO DO controls wise
3. [important] Smooth Cancel Actions - Emergency stop **done**
    - Current is unsafe 
4. [important] Add Visualization & Telemetry **WIP** (perception integration)
    - Plotjuggler **done**
    - Robot TF
    - Rviz  
6. [important] Ensure MPC is solved in specified time (debug messages)
    - Use ros2 rate **done**
    - Informed Initialization **already done**
    - Statistics on solve time
7. Quality of life
    - Trajectory Tracking Improvement
    - Trajectory Smoother Reference
    - Better file structure
    - Handle Failures in initialization
    - Load already compiled mpc model in mpc 
    - Add Mattias RT - Visualization for post processing
