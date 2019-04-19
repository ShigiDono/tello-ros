cmake_minimum_required(VERSION 2.8.3)
project(tello_driver)

## Find catkin and everything we need in ROS
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  rospy
  sensor_msgs
  std_msgs
  tello_msgs
)

## Declare a catkin package and export dependencies
catkin_package(
  CATKIN_DEPENDS
  cv_bridge
  geometry_msgs
  rospy
  sensor_msgs
  std_msgs
  tello_msgs
)
