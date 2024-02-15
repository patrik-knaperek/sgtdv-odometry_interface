/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Patrik Knaperek
/*****************************************************/


#include <ros/ros.h>
#include "../include/odometry_interface.h"
#include <sgtdv_msgs/CarPose.h>
#include <sgtdv_msgs/CarVel.h>
//#include  IMU msg

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_estimate");
  ros::NodeHandle handle;

  ros::Publisher posePublisher = handle.advertise<sgtdv_msgs::CarPose>("pose_estimate", 1);
  ros::Publisher velocityPublisher = handle.advertise<sgtdv_msgs::CarVel>("velocity_estimate", 1);

  OdometryInterface odometry_interface(posePublisher, velocityPublisher);

  // ros::Subscriber slamSub = handle.subscribe("slam_pose", 1, &OdometryInterface::DoSlamState, &odometry_interface);
  //ros::Subscriber imuSub = handle.subscribe("imu", 1, &OdometryInterface::DoIMU, &odometry_interface);
  // ros::Subscriber cameraSub = handle.subscribe("camera_pose", 1, &OdometryInterface::DoCameraPose, &odometry_interface);
  ros::Subscriber odomSub = handle.subscribe("odometry/filtered", 1, &OdometryInterface::doOdometry, &odometry_interface);

  ros::spin();

  return 0;
}
