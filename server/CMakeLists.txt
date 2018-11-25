cmake_minimum_required(VERSION 2.8.3)
project(imaging_ros_ingester)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  image_transport
  cv_bridge
  inertial_sense
)

find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sniper_cam
  CATKIN_DEPENDS rospy inertial_sense sensor_msgs image_transport cv_bridge 
#  DEPENDS system_lib
)


catkin_python_setup()

include_directories(include)
include_directories(
   ${catkin_INCLUDE_DIRS}
)
