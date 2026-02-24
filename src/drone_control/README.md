# Setup

Clone this branch recursively and build+run the docker container. When rebuilding, delete the old image and clear the cache (```docker system prune```).
```bash
git clone --branch temp/controls --recurse-submodules git@github.com:ETHRoboticsClub/ETHRC-Drone-Racing.git
cd ETHRC-Drone-Racing/
docker compose build
docker compose up -d
```

Open 4 terminals and enter the container in each one.
```bash
docker exec -it erc-aeronautics bash
```

## Terminal 1
Run gazebo
```bash
cd ~/ros_ws/px4/PX4-Autopilot && make px4_sitl gz_x500
```
## Terminal 2
Build ROS, then launch the drone controller
```bash
sudo colcon build
source install/setup.bash
ros2 launch drone_control drone_controller.launch.py
```

## Terminal 3
Send the goal to lift off (x=0, y=0, z<=-2)
```bash
source install/setup.bash
ros2 run drone_control send_pose_goal goal_only -x 0.0 -y 0.0 -z -2.0 -yaw 0.0
```

## Terminal 4
To cancel a goal 
```bash
source install/setup.bash
ros2 run drone_control send_pose_goal cancel
# If rejected: 
# ros2 service call /arm_drone std_srvs/srv/SetBool "{data: true}"
# Then retry
```

## Terminal 2
The visualization should be launched after the first goal has been set. An ugly but working solution is to stop and relaunch the drone_control.
```bash
# CTRL+C
ros2 launch drone_control drone_controller.launch.py
```
In the PlotJuggler at "Layout", load ```src/drone_control/config/control_plotjuggler.xml```

## Remarks
- The drone starts from the ground. It is therefore necessary to ensure the first goal does not contain changes in the x and y coordinates which could lead to it leaning into the ground immediately.
- The drone sometimes disarms itself automatically. If a pose goal is rejected by the server, reactivate the drone with ```ros2 service call /arm_drone std_srvs/srv/SetBool "{data: true}"```


# Run the node
This is part of the old README. Expect some parts to be redundant with the "Setup" section above.

This runs the mpc controller && the plotjuggler visualization
```sh
ros2 launch drone_control drone_controller.launch.py 
```

You need to run the simulation first:
```sh
cd ~/ros_ws/px4/PX4-Autopilot && make px4_sitl gz_x500
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
3. [important] Smooth Cancel Actions - Emergency stop
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