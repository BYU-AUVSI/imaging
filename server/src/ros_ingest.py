#! /usr/bin/env python

import rospy
from dao import DAO
from inertial_sense.msg import GPS
from gps_msg import GpsMsg

class RosIngester:

    dao_ = None

    def __init__(self):
        print("Startup ros ingester...")
        self.dao_ = DAO()
        self.gps_subscriber_ = rospy.Subscriber('/gps', GPS, self.gpsCallback, queue_size=10)
        self.gps_msg_ = GpsMsg()
        # self.raw_subscriber = rospy.Subscriber('/other_camera/image_raw/compressed', compressedImage)
        print("Ingester is all setup!")
        # gps type == inertial_sense/GPS
        # image == sensor_msgs/CompressedImage


    def gpsCallback(self, msg):
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
