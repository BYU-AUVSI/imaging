cmake_minimum_required(VERSION 2.8.3)
project(imaging_ros_handler)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  inertial_sense
  uav_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS rospy inertial_sense sensor_msgs uav_msgs
)


catkin_python_setup()

include_directories(include)
include_directories(
   ${catkin_INCLUDE_DIRS}
)
