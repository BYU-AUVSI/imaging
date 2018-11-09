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
from config import config

class RosIngester:

    dao_ = None

    def __init__(self):
        print("Startup ros ingester...")
        configPath = rospy.get_param('~config_path', '../conf/config.ini')
        self.dao_ = DAO(configPath)
        self.gps_subscriber_ = rospy.Subscriber('/gps', GPS, self.gpsCallback, queue_size=10)
        self.gps_msg_ = GpsMsg()
        self.img_subscriber_ = rospy.Subscriber("/other_camera/image_raw/compressed", CompressedImage, self.imgCallback,  queue_size = 10)
        
        params = config(configPath, 'Images')
        print("params:: {}".format(params))
        basePath = params['basedir']
        print("baseDir:: {}".format(basePath))

        self.raw_path_  = basePath + "/raw/"
        self.crop_path_ = basePath + "/crop/"
        # create paths for where images will be dumped
        if not os.path.exists(self.raw_path_):
            os.makedirs(self.raw_path_)
        if not os.path.exists(self.crop_path_):
            os.makedirs(self.crop_path_)
        print("Ingester is all setup!")


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
        cv2.imwrite(self.raw_path_ + filename, cv2.imdecode(rawData, 1))

def main():
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
