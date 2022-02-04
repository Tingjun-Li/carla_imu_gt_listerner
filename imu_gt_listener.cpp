#include "ros/ros.h"
#include <string>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <limits>
#include <vector>
#include <boost/bind.hpp>
// #include <mutex>
// #include <thread>
// #include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include "nav_msgs/Odometry.h"
#include <nav_msgs/Path.h>
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <nav_msgs/Path.h>
#include <random>
#include "imu_gt_listener.hpp"

typedef std::numeric_limits< double > dbl;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");

  ros::NodeHandle nh;

  std::cout << "Ready to subscribe to /carla/ego_vehicle/imu topic" << std::endl;

  CarlaListener carla_listener(nh);
  
  ros::spin();

  return 0;
}