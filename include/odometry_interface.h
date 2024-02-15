/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Patrik Knaperek
/*****************************************************/

#include <ros/ros.h>
#include <sgtdv_msgs/CarPose.h>
#include <sgtdv_msgs/CarVel.h>
//#include  IMU msg
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

class OdometryInterface
{
public:
  OdometryInterface(ros::NodeHandle& nh);
  ~OdometryInterface() = default;

  void doSlamState(const sgtdv_msgs::CarPose::ConstPtr &msg);
  // void DoIMU(/*imu msg*/);
  void doCameraPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  void doOdometry(const nav_msgs::Odometry::ConstPtr &msg);

private:
  ros::Publisher pose_pub_, velocity_pub_;
  ros::Subscriber odometry_sub_;
  ros::Subscriber camera_pose_sub_;

  sgtdv_msgs::CarPose car_pose_msg_;
  sgtdv_msgs::CarVel car_vel_msg_;
};