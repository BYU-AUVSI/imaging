#! /usr/bin/env python

import rospy
import cv2
import numpy as np
import os, time # for img saving
from dao.incoming_image_dao import IncomingImageDAO
from dao.incoming_gps_dao import IncomingGpsDAO
# ROS messages:
from inertial_sense.msg import GPS
from sensor_msgs.msg import CompressedImage
from dao.model.incoming_gps import incoming_gps
from dao.model.incoming_image import incoming_image

class RosIngester:

    def __init__(self):
        print("Startup ros ingester...")
        currentPath = os.path.dirname(os.path.realpath(__file__)) 
        configPath = rospy.get_param('~config_path', currentPath + '/../conf/config.ini')
        startTs = str(int(time.time()))

        self.img_dao_ = IncomingImageDAO(configPath)
        self.gps_dao_ = IncomingGpsDAO(configPath)
        self.gps_subscriber_ = rospy.Subscriber('/gps', GPS, self.gpsCallback, queue_size=10)
        self.gps_msg_ = incoming_gps()
        self.img_subscriber_ = rospy.Subscriber("/other_camera/image_raw/compressed", CompressedImage, self.imgCallback,  queue_size = 10)
        self.img_msg_ = incoming_image()
        self.img_msg_.manual_tap = False
        self.img_msg_.autonomous_tap = False
        
        basePath = currentPath + '/../images/' + startTs
        print("Base dir for images:: {}".format(basePath))

        self.raw_path_  = basePath + "/raw/"
        # create paths for where raw images will be dumped
        if not os.path.exists(self.raw_path_):
            os.makedirs(self.raw_path_)
        print("Ingester is all setup!")


    def gpsCallback(self, msg):
        """
        Ros subscriber callback. Subscribes to the inertial_sense GPS msg.
        Get the lla values from the GPS message and then pass them to the DAO so 
        they can be inserted into the database
        """
        if msg.fix_type == GPS.GPS_STATUS_FIX_TYPE_NO_FIX or msg.num_sat == 0:
            return
	    
        self.gps_msg_.time_stamp = msg.header.stamp.to_sec()
        self.gps_msg_.lat  = msg.latitude
        self.gps_msg_.lon  = msg.longitude
        self.gps_msg_.alt  = msg.altitude 

        # insert into db:
        resultingId = self.gps_dao_.addGps(self.gps_msg_)
        if resultingId == -1:
            print("FAILED to insert gps measurement:")
            print("ts: {}, lat: {}, lon: {}, alt: {}".format(*self.gps_msg_.insertValues()))


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
        fullPath = self.raw_path_ + str(ts) + ".jpg"
        cv2.imwrite(fullPath, cv2.imdecode(rawData, 1))

        self.img_msg_.time_stamp = ts 
        self.img_msg_.image_path = fullPath

        # insert into the db - returns db id of inserted image
        resultingId = self.img_dao_.addImage(self.img_msg_)
        if resultingId == -1:
            print("FAILED to insert image:")
            print("ts: {}, path: {}, manual_tap: {}, autonomous_tap: {}".format(*self.img_msg_.insertValues()))

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
