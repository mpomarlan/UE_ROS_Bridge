cmake_minimum_required(VERSION 2.8.3)
project(UE_ROS_Bridge)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  tf
)


catkin_package(
  CATKIN_DEPENDS rospy std_msgs tf
)

