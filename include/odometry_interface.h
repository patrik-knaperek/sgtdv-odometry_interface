/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Patrik Knaperek
/*****************************************************/

#pragma once

/* ROS */
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

/* SGT */
#include <sgtdv_msgs/CarPose.h>
#include <sgtdv_msgs/CarVel.h>

// #define CAMERA_POSE_INTERFACE

class OdometryInterface
{
public:
  OdometryInterface(ros::NodeHandle& nh);
  ~OdometryInterface() = default;

#ifdef CAMERA_POSE_INTERFACE
  void doCameraPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
#else
    void doOdometry(const nav_msgs::Odometry::ConstPtr &msg);
#endif


private:
  ros::Publisher pose_pub_, velocity_pub_;

#ifdef CAMERA_POSE_INTERFACE
  ros::Subscriber camera_pose_sub_;
#else
  ros::Subscriber odometry_sub_;
#endif

  sgtdv_msgs::CarPose car_pose_msg_;
  sgtdv_msgs::CarVel car_vel_msg_;
};