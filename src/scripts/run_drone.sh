cd ../..
colcon build --packages-select drone_mpc_control
source install/setup.bash
ros2 launch drone_mpc_control drone_mpc_mavsdk.launch.py