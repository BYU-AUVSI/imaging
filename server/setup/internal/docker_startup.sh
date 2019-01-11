#!/bin/bash
# this script runs on the docker image and tells it what todo when it starts up
service postgresql start # start postgres
source /opt/catkin_ws/devel/setup.sh # source our dev workspace
export ROS_HOSTNAME=`hostname -I`
/usr/bin/supervisord # start supervisor, which starts the server and ros_ingestor in the background