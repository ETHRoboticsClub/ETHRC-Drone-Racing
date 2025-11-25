# Building the package

We need to downgrade the `setuptools` version we currently use
```sh
sudo python3 -m pip uninstall -y setuptools
sudo python3 -m pip install setuptools==79.0.1
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
```sh
export PYTHONPATH=$PYTHONPATH:~/ros_ws/src
ros2 run drone_control drone_controller_node
```

You need to run the simulation first:
```sh
cd ~/ros_ws/px4/PX4-Autopilot && make px4_sitl gz_x500
```

# TO DO
1. [important] Better file structure to avoid exporting pythonpath
2. [important] Improve asyncio event loop logic
3. Handle Failures in initialization
4. Load already compiled model in mpc 
5. [important] Add trajectory tracking
6. [important] Ensure MPC is solved in specified time