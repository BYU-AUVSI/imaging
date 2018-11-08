#! /usr/bin/env python

import rospy
import cv2
import numpy as np
import os, sys # for img saving
from dao import DAO
# ROS messages:
from inertial_sense.msg import GPS
from sensor_msgs.msg import CompressedImage
from gps_msg import GpsMsg

class RosIngester:

    dao_ = None

    def __init__(self):
        print("Startup ros ingester...")
        self.dao_ = DAO()
        self.gps_subscriber_ = rospy.Subscriber('/gps', GPS, self.gpsCallback, queue_size=10)
        self.gps_msg_ = GpsMsg()
        self.subscriber = rospy.Subscriber("/other_camera/image_raw/compressed", CompressedImage, self.imgCallback,  queue_size = 10)
        
        # TODO: Remove me and use config file path
        self.pathTmp="/home/len0rd/bags/img/raw/"
        if not os.path.exists(self.pathTmp):
            os.makedirs(self.pathTmp)
        print("Ingester is all setup!")
        # gps type == inertial_sense/GPS
        # image == sensor_msgs/CompressedImage


    def gpsCallback(self, msg):
        """
        Ros subscriber callback. Subscribes to the inertial_sense GPS msg.
        Get the lla values from the GPS message and then pass them to the DAO so 
        they can be inserted into the database
        """
        if msg.fix_type == GPS.GPS_STATUS_FIX_TYPE_NO_FIX or msg.num_sat == 0:
            print("No GPS fix, cant add to db")
            return
	    
        self.gps_msg_.time = msg.header.stamp.to_sec()
        self.gps_msg_.lat  = msg.latitude
        self.gps_msg_.lon  = msg.longitude
        self.gps_msg_.alt  = msg.altitude 

        print("{} :: {} {} {}".format(self.gps_msg_.time, self.gps_msg_.lat, self.gps_msg_.lon, self.gps_msg_.alt))

        # call dao with gps msg

    def stateCallback(self, msg):
        print("state callback")

    def imgCallback(self, msg):
        """
        Ros subscriber callback. Subscribes to the cameras image topic. Saves
        the image file, and passes the corresponding filename and TS to the DAO
        so that it can be inserted into the database
        """
        #get raw img data:
        rawData = np.fromstring(msg.data, np.uint8)
        ts = msg.header.stamp.to_nsec()
        print("img callback: {}".format(ts))
        filename = str(ts) + ".jpg"
        cv2.imwrite(self.pathTmp + filename, cv2.imdecode(rawData, 1))

def main():
    print("Heyyyy")
    # initialize the node
    rospy.init_node('imaging_ingester')

    subscriber = RosIngester()
    # spin
    try:
        rospy.spin()
    except KeyBoardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()
