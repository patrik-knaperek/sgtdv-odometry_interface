/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Patrik Knaperek
/*****************************************************/

#pragma once

/* ROS */
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>

/* SGT */
#include <sgtdv_msgs/CarPose.h>
#include <sgtdv_msgs/CarVel.h>

// #define CAMERA_POSE_INTERFACE

class OdometryInterface
{
public:
  explicit OdometryInterface(ros::NodeHandle& nh);
  ~OdometryInterface() = default;

private:
#ifdef CAMERA_POSE_INTERFACE
  void doCameraPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
#else
    void doOdometry(const nav_msgs::Odometry::ConstPtr &msg);
#endif

  bool resetOdomCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  ros::Publisher pose_pub_, velocity_pub_;
  
  ros::ServiceServer reset_odom_server_;
  ros::Publisher vesc_reset_odom_pub_;
  std::string camera_reset_odom_client_;

#ifdef CAMERA_POSE_INTERFACE
  ros::Subscriber camera_pose_sub_;
#else
  ros::Subscriber odometry_sub_;
#endif

  sgtdv_msgs::CarPose car_pose_msg_;
  sgtdv_msgs::CarVel car_vel_msg_;
  std_srvs::Empty empty_srv_;
};