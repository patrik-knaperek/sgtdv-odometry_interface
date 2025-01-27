/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Patrik Knaperek
/*****************************************************/

/* ROS */
#include <tf/tf.h>
#include <std_msgs/Empty.h>

/* Header */
#include "odometry_interface.h"

OdometryInterface::OdometryInterface(ros::NodeHandle& nh)
  /* ROS interface init */
  : pose_pub_(nh.advertise<sgtdv_msgs::CarPose>("odometry/pose", 1))
  , velocity_pub_(nh.advertise<sgtdv_msgs::CarVel>("odometry/velocity", 1))
  , reset_odom_server_(nh.advertiseService("/reset_odometry", &OdometryInterface::resetOdomCallback, this))
  , vesc_reset_odom_pub_(nh.advertise<std_msgs::Empty>("vesc/vesc_to_odom/reset_odometry", 1))
  , camera_reset_odom_client_("/camera/reset_odometry")

#ifdef CAMERA_POSE_INTERFACE
  , camera_pose_sub_(nh.subscribe("camera/pose", 1, &OdometryInterface::doCameraPose, this))
#else
  , odometry_sub_(nh.subscribe("odometry/filtered", 1, &OdometryInterface::doOdometry, this))
#endif
{
}

#ifdef CAMERA_POSE_INTERFACE
void OdometryInterface::doCameraPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  car_pose_msg_.position.x = msg->pose.pose.position.x;
  car_pose_msg_.position.y = msg->pose.pose.position.y;

  tf::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);

  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  car_pose_msg_.yaw = yaw;
    
  pose_pub_.publish(car_pose_msg_);
}
#else
void OdometryInterface::doOdometry(const nav_msgs::Odometry::ConstPtr &msg)
{
  car_pose_msg_.position.x = msg->pose.pose.position.x;
  car_pose_msg_.position.y = msg->pose.pose.position.y;

  tf::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);

  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  car_pose_msg_.yaw = yaw;

  car_vel_msg_.speed = msg->twist.twist.linear.x;
  car_vel_msg_.yaw_rate = msg->twist.twist.angular.z;

  pose_pub_.publish(car_pose_msg_);
  velocity_pub_.publish(car_vel_msg_);
}
#endif

bool OdometryInterface::resetOdomCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  ROS_INFO("Resetting odometry");
  
  if(!ros::service::call(camera_reset_odom_client_, empty_srv_))
  {
    ROS_WARN_STREAM("Service \"" << camera_reset_odom_client_ << "\" failed");
  }

  vesc_reset_odom_pub_.publish(std_msgs::Empty());

  return true;
}
