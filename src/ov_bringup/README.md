### Run OpenVINS (ROS 2) 

```bash
# Build the workspace
colcon build --event-handlers console_cohesion+ --packages-select ov_core ov_init ov_msckf ov_eval 
source install/setup.bash

# Launch OpenVINS with test V1_01_easy bag
ros2 launch ov_msckf subscribe.launch.py config:=euroc_mav

# Play the rosbag 
ros2 bag play V1_01_easy
```
