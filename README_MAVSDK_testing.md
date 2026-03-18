Install the new docker:

```
git clone --branch temp/controls --recurse-submodules git@github.com:ETHRoboticsClub/ETHRC-Drone-Racing.git
cd ETHRC-Drone-Racing/
docker compose build
docker compose up -d
```

On the host (outside docker)
```
ls -l /dev/serial/by-id/
```
Make sure PX4 is detected, also save the port: If cable is USBC to USBC you should see something like:
```
usb-Holybro_PX4_KakuteH7_0-if00 -> ../../ttyACM0
```
Otherwise, ->../../ttyUSB0, or USB1, USB2. Verify that you also detect the usb port inside the docker (important)

```
docker compose exec dev bash
lsusb
```
You have to see Holybro PX4 KakuteH7

Every command should be inside docker


Terminal 1:
Run the mavsdk (offboard mode), **replace** ACM with your port.
```
source install/setup.bash
ros2 run my_mavsdk px4_attitude_rate_offboard --ros-args  -p rate_hz:=50.0 -p system_address:="serial:///dev/ttyACM0:115200"
```
On another node arm the drone, if the motors dont move (something is wrong, you cant continue):

Terminal 2:
```
source install/setup.bash
ros2 service call /arm_drone std_srvs/srv/SetBool "{data: true}"

```

Terminal 3:
Send command topics, you can either run a script that publish topics T=offset+sin(f*t), recommend to keep T<0.5, otherwise motors stop (safety reasons, TO DO).

```
source install/setup.bash
ros2 run my_mavsdk sin_thrust_publisher --ros-args   -p amplitude:=0.1 -p offset:=0.1 -p frequency:=1
```
either send commands directly:
```
source install/setup.bash
ros2 topic pub /offboard/attitude_rate_cmd std_msgs/msg/Float32MultiArray   "{data: [0.0, 0.0, 0.0, 0.2]}"
```

On a new terminal (Terminal 4,5)
**READ DATA**

Change the port
*ROTOR*
```
ros2 run my_mavsdk px4_motor_torque_publisher --ros-args -p rate_hz:=100.0 -p system_address:="serial:///dev/ttyACM0:115200"
```
*IMU* (max frequency 70, TODO: go to 200)
```
ros2 run my_mavsdk px4_imu_publisher --ros-args -p rate_hz:=70.0 -p system_address:="serial:///dev/ttyACM0:115200"
```

Visualize data (Terminal 6):
```
source install/setub.bash
ros2 run plotjuggler plotjuggler
```
```
source install/setub.bash
ros2 topic list
```
