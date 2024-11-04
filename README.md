# **OdometryInterface package**

___

© **SGT Driverless**

**Authors:** Patrik Knaperek

**Objective:** Translation of odometry topic into separate pose and velocity topics
___

### Related packages
* [`robot_localization`](../robot_localization/README.md)
* [`camera_driver`](../camera_driver/README.md)

### Topic conversions
* `/odometry/filtered [nav_msgs/Odometry]` →  `/slam/pose [sgtdv_msgs/CarPose]`, `/odometry/velocity [sgtdv_msgs/CarVel]`
* (alternatively) `/camera/pose [geometry_msgs/PoseWithCovarianceStamped]` → `/odometry/pose [sgtdv_msgs/CarPose]`


## Compilation
```sh
$ cd ${SGT_ROOT}/ros_implementation
$ catkin build odometry_interface
```

### Compilation configuration
* [`odometry_interface.h`](./include/odometry_interface.h)
  * `CAMERA_POSE_INTERFACE` : uncomment to use as interface for visual odometry instead of odometry fusion (robot_localization)

## Launch

### Source the environment
```sh
$ cd ${SGT_ROOT}/ros_implementation
$ source ./devel/setup.bash
```
### Launch the node
* standalone
```sh
$ rosrun odometry_interface odometry_interface
```
* along with other nodes on RC car
```sh
$ roslaunch master rc.launch
```
