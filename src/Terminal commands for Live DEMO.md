# Terminal commands for the Live Demonstration

## Wall-follower
Acces the Python script for each control (located at *my_rUBot_mecanum/src/my_robot_control/my_robot_control/*) and execute them.

1. **Differential** control 
   ```
   /usr/bin/python3/home/student/Desktop/my_rUBot_mecanum/src/my_robot_control/my_robot_control/my_robot_wallfollower_differential.py
   ```

2. **Holonomic** control
   ```
   /usr/bin/python3/home/student/Desktop/my_rUBot_mecanum/src/my_robot_control/my_robot_control/my_robot_wallfollower_holonomic.py
   ```

3. **Holonomic Improved** control
   ```
   /usr/bin/python3/home/student/Desktop/my_rUBot_mecanum/src/my_robot_control/my_robot_control/my_robot_wallfollower_holonomic_obstacle_V2.py
   ```

----
## Navigation
1. Reset odomoetry
   ```
   ros2 topic pub --once /reset_odom std_msgs/msg/Bool "{data: true}"
   ```

2. Launch the navigation stack
   ```
   ros2 launch my_robot_navigation2 navigation2_robot.launch.py use_sim_time:=false map_file:=map_V2_B2.yaml params_file:=rubot_real_lidar_nav.yaml
   ```

3. Launch the Python node with the Waypoints
   ```
   ros2 launch my_robot_nav_control nav_waypoints.launch.py wp_file:=B2_waypoints_hw.yaml
   ```

----
## AI traffic sign
1. Reset odomoetry
   ```
   ros2 topic pub --once /reset_odom std_msgs/msg/Bool "{data: true}"
   ```
   
2. Launch the navigation stack
   ```
   ros2 launch my_robot_navigation2 navigation2_robot.launch.py use_sim_time:=false map_file:=map_V2_B2.yaml params_file:=rubot_real_lidar_nav.yaml
   ```
   
3. Add the classifier
   ```
   ros2 launch my_robot_ai_perception traffic_sign_classifier.launch.py config_file:=traffic_sign_classifier_real.yaml
   ```
   
4. Launch the navaigation
   ```
   ros2 launch my_robot_ai_navigation traffic_sign_navigation.launch.py config_file:=traffic_sign_navigation_real.yaml
   ```
   
5. Verify the classified traffic sign
   ```
   ros2 topic echo /traffic_sign
   ```
