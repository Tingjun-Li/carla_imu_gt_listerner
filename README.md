# carla_imu_gt_listerner
This repository can output imu data with noise and Ground truth pose received from Carla
1. Put the repo into catkin_ws/src/
2. Run `catkin_make` under catkin_ws/
3. Run `source devel/setup.bash`
4. Open a new terminal and run `roscore`
5. Then go back to the original terminal and run `rosrun carla_path carla_listenser_node` under catkin_ws/
