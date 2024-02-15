/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský, Patrik Knaperek
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
    OdometryInterface(const ros::Publisher& posePublisher, const ros::Publisher& velocityPublisher);
    ~OdometryInterface() = default;

    void doSlamState(const sgtdv_msgs::CarPose::ConstPtr &msg);
    // void DoIMU(/*imu msg*/);
    void doCameraPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void doOdometry(const nav_msgs::Odometry::ConstPtr &msg);
private:
    ros::Publisher pose_pub_, velocity_pub_;
    sgtdv_msgs::CarPose car_pose_msg_;
    sgtdv_msgs::CarVel car_vel_msg_;
};