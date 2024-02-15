/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Patrik Knaperek
/*****************************************************/

/* ROS */
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

/* SGT */
#include <sgtdv_msgs/CarPose.h>
#include <sgtdv_msgs/CarVel.h>

class OdometryInterface
{
public:
  OdometryInterface(ros::NodeHandle& nh);
  ~OdometryInterface() = default;

  void doOdometry(const nav_msgs::Odometry::ConstPtr &msg);
  void doCameraPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);


private:
  ros::Publisher pose_pub_, velocity_pub_;
  ros::Subscriber odometry_sub_;
  // ros::Subscriber camera_pose_sub_;

  sgtdv_msgs::CarPose car_pose_msg_;
  sgtdv_msgs::CarVel car_vel_msg_;
};