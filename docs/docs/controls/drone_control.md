# Drone Control

**Page Owner:** [Michalis Papadakis](https://github.com/MichalisPapadakis)

---

## Overview

This node implements the drone controller that receives the **estimated system state** and executes pose or trajectory goals via ROS 2 actions.

**Important**: The node is intented to be controller agnostic, so:
1. multiple controllers can utilize the same interfaces
2. multiple controllers can handle different goals (e.g. LQR for hovering, MPC/RL for trajectory tracking, PID for landing etc)

To achieve this, the node has 2 state flags: `active_goal` that indicates if the action server serves an action, and `hovering_active` that indicates if the controller has entered a hovering state, where it stabilizes the current state. When executing a particular goal, the controller stops hovering and uses the controller associated with the particular action. 

**Note**: Currently the controller only supports MPC for hovering, pose and trajectory goals, and needs modifications to support other controllers. The action interface is intented for task-specific controllers, the code needs to be improved for that. 

All the parameters are configured in [drone_controller_config.yaml](https://github.com/ETHRoboticsClub/ETHRC-Drone-Racing/blob/main/src/drone_control/config/drone_controller_config.yaml)

## Action Interfaces

The node exposes the following ROS 2 action servers for goal-based control. 

### Set Pose Goal Action
- **Action Name**: `/drone_set_pose_goal`
- **Type**: `FollowWaypoints`
- **Description**: Commands the drone to move to a single target pose
- **Behavior**:
  - Accepts a pose goal only if currently no other goal is active
  - Disables hovering controller so pose controller can be activated
  - Executes closed-loop control until the target is reached with a specified threshold (specified by the `threshold` parameter) or the goal is canceled. 
  - Finishing the execution, it launches a *hovering* thread that stabilizes the current state.

### Set Trajectory Goal Action
- **Action Name**: `/drone_set_trajectory_goal`
- **Type**: `FollowWaypoints`
- **Description**: Commands the drone to follow a sequence of waypoints (trajectory)
- **Behavior**:
  - Accepts a list of waypoints only if currently no other goal is active
  - Disables hovering controller so pose controller can be activated
  - Executes closed-loop control until the target is reached with a specified threshold (specified by the `threshold` parameter) or the goal is canceled. 
  - Finishing the execution, it launches a *hovering* thread that stabilizes the current state.

The helper nodes: `send_pose_goal` and `send_trajectory_goal` showcase the use of the action inteface and its pre-emption capabilities from the CLI and can be used for developement. Their used is shown [below](#run-the-node) However, the GUI is the intented action client. 

## Input Topics

This controller node subscribes to the following ROS 2 topics:

### Drone State
- **Topic**: specified by `state_topic` parameter. (Default: `drone_state_sim`)
    - `\drone_state_sim`: For getting the state from mavsdk
    - `<perception_topic>`: For getting the state from perception pipeline
- **Type**: `std_msgs/msg/Float64MultiArray`
- **Description**: Current drone state vector (e.g., position, velocity, quaternion, angular velocity)
- **Frequency**: sim: 200Hz
- **Queue Size**: 10

---

## Command Topics

The controller node publishes the following ROS 2 topics:

### AttitudeRate Command
- **Topic**: specified by `attitude_command_topic` parameter. (Default: `/drone_attitude_cmd`)
- **Type**: `std_msgs/msg/Float64MultiArray`
- **Description**: Attitude rate command sent to the drone (e.g., roll-rate, pitch-rate, yaw-rate, normalized-thrust). See [AttitudeRate](https://mavsdk.mavlink.io/main/en/cpp/api_reference/structmavsdk_1_1_offboard_1_1_attitude_rate.html)
- **Frequency**: specified by `controller_rate` parameter (Default: 100hz)
- **Queue Size**: 10

### Position Command
- **Topic**: specified by `position_command_topic` parameter. (Default: `drone_position_cmd`)
- **Type**: `std_msgs/msg/Float64MultiArray`
- **Description**: Position command sent in the earth NED frame. See [PositionNedYaw](https://mavsdk.mavlink.io/main/en/cpp/api_reference/structmavsdk_1_1_offboard_1_1_attitude_rate.html)
- **Frequency**: specified by `controller_rate` parameter (Default: 100hz)
- **Queue Size**: 10

## Other Output Topics

### Current Target Pose
This is used to visualize the active pose target in rviz and telemetry
- **Topic**: `/drone_current_target`
- **Type**: `geometry_msgs/msg/PoseStamped`
- **Description**: Current target pose that the controller is tracking (e.g., active waypoint)
- **QoS**:
  - Durability: `TRANSIENT_LOCAL`
  - Depth: 1
- **Frequency**: Updated on target changes or during trajectory execution


## Run the node
1. Run the simulation first:
    ```sh
    cd ~/ros_ws/px4/PX4-Autopilot && make px4_sitl gz_x500
    ```

2. Run the mpc controller, mavsdk client && plotjuggler visualization
    ```sh
    ros2 launch drone_control drone_controller.launch.py 
    ```

3. You can test the  controller from CLI with (this is a placeholder node, since this functionalities will be implemented using GUI):

    **Important**: Make sure drone is armed. At startup, if one does not give a command, it disarms automatically. One can arm it by:
    ```sh
    ros2 service call /arm_drone std_srvs/srv/SetBool "{data: true}"
    ```

    **Position goals:**
    ```sh
    ros2 run drone_control send_pose_goal goal_only -x 0.0 -y 0.0 -z -1.0 -yaw 0.0
    ```

    **Trajectory goals:**
    1. Modify the [yaml_file](https://github.com/ETHRoboticsClub/ETHRC-Drone-Racing/blob/main/src/drone_control/config/waypoints_config.yaml)
    2. Run
    ```sh
    ros2 run drone_control send_trajectory_goal goal_only
    ```



    ## TODO:
    
    1. Merge with perception
    2. Use [Actuator Commands](https://mavsdk.mavlink.io/main/en/cpp/api_reference/structmavsdk_1_1_offboard_1_1_actuator_control.html) 
    3. Use OOP to abstract the controller class and allow different controllers with different interfaces to run simultaneously. 
    4. Debugging - Better Telemetry - Controller Statistics

