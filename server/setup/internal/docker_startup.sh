#!/bin/bash
service postgresql start
source /opt/catkin_ws/devel/setup.sh
# export ROS_HOSTNAME=`hostname -I`
/usr/bin/supervisord
# rosrun imaging_ros_ingester ros_ingest.py