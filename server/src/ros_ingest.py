#! /usr/bin/env python

import rospy
import cv2
import numpy as np
import os, time # for img saving
from dao.base_dao import BaseDAO
# ROS messages:
from inertial_sense.msg import GPS
from sensor_msgs.msg import CompressedImage
from dao.model.gps_msg import GpsMsg
from dao.model.incoming_image import IncomingImage

class RosIngester:

    dao_ = None

    def __init__(self):
        print("Startup ros ingester...")
        currentPath = os.path.dirname(os.path.realpath(__file__)) 
        configPath = rospy.get_param('~config_path', currentPath + '/../conf/config.ini')
        startTs = str(int(time.time()))

        self.dao_ = BaseDAO(configPath)
        self.gps_subscriber_ = rospy.Subscriber('/gps', GPS, self.gpsCallback, queue_size=10)
        self.gps_msg_ = GpsMsg()
        self.img_subscriber_ = rospy.Subscriber("/other_camera/image_raw/compressed", CompressedImage, self.imgCallback,  queue_size = 10)
        self.img_msg_ = IncomingImage()
        
        basePath = currentPath + '/../images/' + startTs
        print("Base dir for images:: {}".format(basePath))

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
        ts = msg.header.stamp.to_sec()
        print("img callback: {}".format(ts))
        fullPath = self.raw_path_ + str(ts) + ".jpg"
        cv2.imwrite(fullPath, cv2.imdecode(rawData, 1))

        self.img_msg_.time = ts 
        self.img_msg_.image_path = fullPath
        self.img_msg_.claimed_manual = False
        self.img_msg_.claimed_autonomous = False

        # call dao with img msg

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
