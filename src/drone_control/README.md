# Building the package

We need to downgrade the `setuptools` version we currently use
```sh
sudo python3 -m pip uninstall -y setuptools
sudo python3 -m pip install setuptools==79.0.1
```

Install nav2 msgs
```sh
sudo apt install ros-humble-nav2-msgs
```

# Fix port is bound:
Disconnect from container and do this:
```sh
$ sudo lsof -i :50051
COMMAND   PID       USER   FD   TYPE DEVICE SIZE/OFF NODE NAME
code    62707 papaveneti   42u  IPv4 762084      0t0  TCP localhost:50051->localhost:39538 (CLOSE_WAIT)
$ kill 62707 # copy the PID 
```

# Run the node
This runs the mpc controller && the plotjuggler visualization
```sh
ros2 launch drone_control drone_controller.launch.py 
```

You need to run the simulation first:
```sh
cd ~/ros_ws/px4/PX4-Autopilot && make px4_sitl gz_x500
```

You can test the action client with (this is a placeholder node, since this functionalities will be implemented using GUI):
```sh
ros2 run drone_control send_pose_goal goal_only -x 0.0 -y 0.0 -z -1.0 -yaw 0.0
```

# TO DO
1. [important] Add actions for setting position and trajectory goals
    - Position **done**
    - Trajectory
    - Cancellation - **WIP**
2. [important] Add trajectory tracking
3. [important] Add Visualization & Telemetry
    - Plotjuggler config **done**
    - Robot TF
    - Rviz  
    - Add plotjuggler & nav2_msgs in docker
    - Add mpc information (e.g. iterations-solve time)
4. [important] Add useful services
    - Lifting off
    - Landing
    - Disarming & arming
5. [important] Ensure MPC is solved in specified time 
    - Use ros2 rate **done**
    - Informed Initialization
    - Statistics on solve time
6. [important] Fix Bug
    - if controller is started without a first command soon, it turns off
6. Quality of life
    - Avoid exporting pythonpath **done**
    - Better Life structure
    - Improve asyncio event loop logic
    - Handle Failures in initialization
    - Load already compiled mpc model in mpc 
    - Add Mattias RT - Visualization for post processing