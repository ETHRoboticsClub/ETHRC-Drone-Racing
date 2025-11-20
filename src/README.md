
### Building the package 

```
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-ignore px4
```

The cmake **Release** option is foundamental for having the best possible computational performance

### Source 
```
source install/setup.bash
```

### Launch OpenVINS
There are multiple parameters to tune, pls look at whole of them to get a good understunding of the pipeline nad get reproducable results
```
ros2 launch ov_msckf subscribe.launch.py
```

Consideration on performance locally, the parameter use_sim_time should be kept as True, otherwise the pipeline tends to drift 

### Play Rosbag
```
ros2 bag play path/to/rosbag
```


