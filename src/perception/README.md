
# Perception 
Perception stack of the autonomous drone racing project at RETH

TBD: drawing of the full perception submodule 

## Building the package 

Go into the root of the workspace and execute the following command:

```
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release 
```

The cmake **Release** option is foundamental for having the best possible computational performance

### Source 
```
source install/setup.bash
```

### Launch OpenVINS

```
ros2 launch ov_msckf subscribe.launch.py
```

Consideration on performance locally, the parameter use_sim_time should be kept as True, otherwise the pipeline tends to drift 




