#First terminal
cd ~/ros_ws/px4/PX4-Autopilot
make px4_sitl gz_x500

#Second terminal
ros2 launch drone_control drone_controller.launch.py

If you want to run the RViz visualizer use 

 ros2 launch drone_control sim.launch.py                                       


 Inside sim.launch.py you can activate the RViz and the plotjuggler visualization here:

# ── Visualization toggles ──────────────────────────────────────────────────────
ENABLE_RVIZ        = True   # RViz2 waypoint/trajectory visualizer
ENABLE_PLOTJUGGLER = False   # PlotJuggler real-time signal plotter
# ──────────────────────────────────────────────────────────────────────────────


#Third terminal
ros2 run drone_control send_trajectory_goal goal_only                                                  


#Somewhere
colcon build --packages-skip \
  aruco_pose_estimation \
  ekf \
  gazebo_ground_thruth \
  ov_core \
  ov_data \
  ov_eval \
  ov_init \
  ov_msckf \
  pointcloud_gt
  )

#Everyhere
source install/setup.bash   