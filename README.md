# ROS2_rUBot_mecanum

In this ROS2 repository we design, simulate and control different robots:
- custom rUBot mecanum robot
- Limo commercial robot
- Paula Pepió's commit contribution to the repository (director).
- David Bonillo's commit contribution to the repository (collaborator).
- Laia Colomer's commit contribution to the repository (collaborator).

## Commands to launch our custom world
1) pkill -f "ign gazebo"; cd ~/my_rUBot_mecanum && colcon build --packages-select my_robot_bringup && source install/setup.bash
2) ros2 launch my_robot_bringup my_robot_bringup_gz.launch.py world:=groupB2_custom.world x:=0.0 y:=-2.4 yaw:=1.5708

## Command to launch the navigation stack Nav2 (with optimized LiDAR configuration):
* ros2 launch my_robot_navigation2 navigation2_robot.launch.py use_sim_time:=true map_file:=groupB2_map.yaml params_file:=rubot_sw_lidar_ign.yaml
