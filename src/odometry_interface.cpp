/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Patrik Knaperek
/*****************************************************/


#include "../include/odometry_interface.h"
#include <tf/tf.h>

OdometryInterface::OdometryInterface(const ros::Publisher& posePublisher, const ros::Publisher& velocityPublisher)
: pose_pub_(posePublisher)
, velocity_pub_(velocityPublisher)
{
}

void OdometryInterface::doSlamState(const sgtdv_msgs::CarPose::ConstPtr &msg)
{
  car_pose_msg_.position = msg->position;
  car_pose_msg_.yaw = msg->yaw;

  pose_pub_.publish(car_pose_msg_);
}

// void OdometryInterface::DoIMU()//imu msg)
// {
//     //m_currentState.position +=
//     //m_currentState.yaw = 

//     SendCarPose();
// }

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