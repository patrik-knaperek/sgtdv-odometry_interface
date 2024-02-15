/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Patrik Knaperek
/*****************************************************/

#include "../include/odometry_interface.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odometry_interface");
  ros::NodeHandle handle;

  OdometryInterface odometry_interface(handle);

  ros::spin();

  return 0;
}
