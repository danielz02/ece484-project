cmake_minimum_required(VERSION 2.8.3)
project(gem_gnss)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
  novatel_gps_msgs
  nav_msgs
  std_msgs
  tf
)

catkin_package(
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


