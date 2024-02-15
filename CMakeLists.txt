cmake_minimum_required(VERSION 3.0.2)
project(odometry_interface)

add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sgtdv_msgs
)

catkin_package(
  CATKIN_DEPENDS sgtdv_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/odometry_interface_node.cpp src/odometry_interface.cpp )

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)


