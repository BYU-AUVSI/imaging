cmake_minimum_required(VERSION 2.8.3)
project(imaging_ros_ingester)

find_package(catkin REQUIRED COMPONENTS
  rospy
  image_transport
  inertial_sense
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sniper_cam
  CATKIN_DEPENDS rospy inertial_sense
#  DEPENDS system_lib
)

include_directories(include)
include_directories(
   ${catkin_INCLUDE_DIRS}
)