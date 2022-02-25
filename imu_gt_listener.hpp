#pragma once
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
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include "nav_msgs/Odometry.h"
#include <nav_msgs/Path.h>
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <nav_msgs/Path.h>
#include <random>
#include "carla_msgs/CarlaEgoVehicleStatus.h"

class CarlaListener {
public:
  CarlaListener(ros::NodeHandle& nh): nh_(nh),
                                    gyro_noise(0.0515698903002, 0.037322230667),
                                    acc_noise(0.515757598072, 0.654051323977) {
    
    imu_sub = nh.subscribe("/carla/ego_vehicle/imu", 1000, &CarlaListener::ImuCallback, this);
    gt_sub = nh.subscribe("/carla/ego_vehicle/odometry", 1000, &CarlaListener::OdomCallback, this);
    velocity_sub = nh.subscribe("/carla/ego_vehicle/vehicle_status", 1000, &CarlaListener::VelocityCallback, this);

    imu_pub = nh_.advertise<sensor_msgs::Imu>("imu_with_noise", 1000);
    path_pub = nh_.advertise<nav_msgs::Path>("gt_path", 1000);
    velocity_pub = nh_.advertise<geometry_msgs::TwistStamped>("carla_velocity", 1000);
  }

  // ~CarlaListener() {}

  void ImuCallback(const sensor_msgs::Imu& imu_msg)
  {

    sensor_msgs::Imu imu_with_noise_msg;

    imu_with_noise_msg.orientation = imu_msg.orientation;
    imu_with_noise_msg.angular_velocity = imu_msg.angular_velocity;

    int noise_factor = 1;
    imu_with_noise_msg.angular_velocity.x += noise_factor * gyro_noise(generator);
    imu_with_noise_msg.angular_velocity.y += noise_factor * gyro_noise(generator);
    imu_with_noise_msg.angular_velocity.z += noise_factor * gyro_noise(generator);

    imu_with_noise_msg.linear_acceleration = imu_msg.linear_acceleration;
    imu_with_noise_msg.linear_acceleration.x += noise_factor * acc_noise(generator);
    imu_with_noise_msg.linear_acceleration.y += noise_factor * acc_noise(generator);
    imu_with_noise_msg.linear_acceleration.z += noise_factor * acc_noise(generator);

    imu_with_noise_msg.header = imu_msg.header;
    imu_pub.publish(imu_with_noise_msg);

  }

  void OdomCallback(const nav_msgs::Odometry& msg)
  {
    geometry_msgs::PoseStamped pose;
    pose.header = msg.header;
    pose.pose = msg.pose.pose;

    if (poses_.size() == 0) {
        initial_pose = pose.pose;
    } 

    pose.pose.position.x -= initial_pose.position.x;
    pose.pose.position.y -= initial_pose.position.y;
    pose.pose.position.z -= initial_pose.position.z;

    poses_.push_back(pose);

    if (poses_.size() % 10 == 0) {
      nav_msgs::Path path_msg;
      path_msg.header = msg.header;
      path_msg.header.frame_id = "/odom";
      std::cout << "Positions: " << poses_.back().pose.position.x << ", " << poses_.back().pose.position.x << ", " << poses_.back().pose.position.z << std::endl;
      path_msg.poses = poses_;

      path_pub.publish(path_msg);
    }
  }

  void VelocityCallback(const carla_msgs::CarlaEgoVehicleStatus& msg)
  {
    geometry_msgs::TwistStamped velocity_msg;
    velocity_msg.header = msg.header;
    velocity_msg.twist.linear.x = msg.velocity;
    velocity_pub.publish(velocity_msg);
    
  }

private:
    ros::NodeHandle& nh_;
    std::vector<geometry_msgs::PoseStamped> poses_;
    ros::Subscriber imu_sub;
    ros::Subscriber gt_sub;
    ros::Subscriber velocity_sub;

    ros::Publisher imu_pub;
    ros::Publisher path_pub;
    ros::Publisher velocity_pub;

    std::default_random_engine generator;
    std::normal_distribution<double> gyro_noise;
    std::normal_distribution<double> acc_noise;
    geometry_msgs::Pose initial_pose;
};

