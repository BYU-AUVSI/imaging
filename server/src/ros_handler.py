#! /usr/bin/env python
import rospy
import cv2
import numpy as np
import os, time # for img saving
from cv_bridge import CvBridge, CvBridgeError # for publishing images saved on server
from config import defaultConfigPath, config
# database access objects
from dao.incoming_image_dao import IncomingImageDAO
from dao.incoming_gps_dao import IncomingGpsDAO
from dao.incoming_state_dao import IncomingStateDAO
from dao.submitted_target_dao import SubmittedTargetDAO
# ROS messages:
from inertial_sense.msg import GPS
from rosplane_msgs.msg import State
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32
from uav_msgs.msg import InteropImage
# model classes for the dao
from dao.model.incoming_gps import incoming_gps
from dao.model.incoming_image import incoming_image
from dao.model.incoming_state import incoming_state
from dao.model.submitted_target import submitted_target

class RosImagingHandler:
    """
    This script is the bridge between ROS and the rest of the imaging system. 
    It has two objectives:
        - listen to the ROS network and save relevant information to the server's database
        - take completed targets from the server's database and send them to interop over ROS
    Subscribes to the raw image, state, gps and focal_length ros topics
    """

    STATE_SAVE_EVERY = 10 # save every 10th state messages (otherwise we get wayyy to many)

    def __init__(self):
        print("Startup ros imaging handler...")
        currentPath = os.path.dirname(os.path.realpath(__file__))
        self.configPath = rospy.get_param('~config_path', defaultConfigPath())
        startTs = str(int(time.time()))

        self.bridge = CvBridge()

        # gps ingestion setup:
        self.gps_dao_ = IncomingGpsDAO(self.configPath)
        self.gps_subscriber_ = rospy.Subscriber('/gps', GPS, self.gpsCallback, queue_size=10)
        self.gps_msg_ = incoming_gps()

        # imaging ingestion setup:
        self.img_dao_ = IncomingImageDAO(self.configPath)
        self.img_subscriber_ = rospy.Subscriber("/a6000_ros_node/img/compressed", CompressedImage, self.imgCallback,  queue_size = 10)
        self.img_msg_ = incoming_image()
        self.img_msg_.focal_length = 16.0 # this is a safe starting assumption == fully zoomed out on a6000
        self.img_msg_.manual_tap = False
        self.img_msg_.autonomous_tap = False

        # focal length ingestion setup (we could roll with a custom msg to 
        # include it with the image msg, but IMO it's better to stick to standard msg types)
        # to reduce dependency tracking.... But you may feel otherwise. Really there's no perfect solution here
        self.fl_subscriber = rospy.Subscriber("/a6000_ros_node/img/focal_length", Float32, self.flCallback,  queue_size = 10)

        # state ingestion setup:
        self.state_dao_ = IncomingStateDAO(self.configPath)
        self.state_subscriber_ = rospy.Subscriber('/state', State, self.stateCallback, queue_size=10)
        self.state_msg_ = incoming_state()
        self.state_interval_ = 0

        # publish for completed targets. Once a target has gone through the entire
        # system, the server puts the finished target in a table and marks it ready for submission
        self.target_pub_ = rospy.Publisher('imaging/target', InteropImage, queue_size=10)
        
        basePath = os.path.abspath(currentPath + '/../images/' + startTs)
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
        """
        Ros subscriber callback. Subscribes to the /state rosplane topic. Passes the roll,
        pitch and yaw angle to be saved by the DAO.
        """
        self.state_interval_ = (self.state_interval_ + 1) % self.STATE_SAVE_EVERY
        if self.state_interval_ != 0:
            return

        ts = msg.header.stamp.to_sec()
        self.state_msg_.time_stamp = ts
        # get rpy ANGLES
        self.state_msg_.roll = msg.phi
        self.state_msg_.pitch = msg.theta
        self.state_msg_.yaw = msg.psi

        resultingId = self.state_dao_.addState(self.state_msg_)
        if resultingId == -1:
            print("FAILED to insert state measurement:")
            print("ts: {}, roll: {}, pitch: {}, yaw: {}".format(*self.state_msg_.insertValues()))

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

    def flCallback(self, msg):
        # take the focal length published on the message and set it to our 
        # incoming_image model
        self.img_msg_.focal_length = msg.data

    def targetToInteropMsg(self, target):
        """
        Convert a submitted_target model to an InteropImage msg
        returns an InteropImage msg ready for publishing
        """

        msgOut = InteropImage()

        #figure out the image part of the message
        if hasattr(target, 'crop_path'):
            cv_img = cv2.imread(target.crop_path)
            try:
                msgOut.image = self.bridge.cv2_to_imgmsg(cv_img, "bgr8")
            except CvBridgeError as e:
                print(e)

        # this dictionary will hold all the values that we care about
        # pushing into the message for whatever target type we're dealing with
        # if stuff is none in the final target, then this will also deal with 
        # checking for that
        targetDict = target.toAuvsiJson()
        
        for key, value in targetDict.items():
            if hasattr(msgOut, key):
                setattr(msgOut, key, value)

        return msgOut

    def publishPendingTargets(self):
        # if there are people actually subscribed to this topic
        if self.target_pub_.get_num_connections() > 0:
            target_dao = SubmittedTargetDAO(self.configPath)
            # if there are targets waiting to be submitted to the judges
            if target_dao.areTargetsPending():
                # then lets submit them
                pending = target_dao.getAllPendingTargets()
                if pending is not None and pending:
                    for target in pending:
                        imageMsg = self.targetToInteropMsg(target)
                        self.target_pub_.publish(imageMsg)

                        target_dao.setTargetSubmitted(target.target, target.autonomous)
                        
            else: # some debug printing
                print("no targets pending")
        else:
            print("nothing subbed to targets")

    def run(self):
        # as per (this very old thread): http://ros-users.122217.n3.nabble.com/ros-spinOnce-equivalent-for-rospy-td2317347.html
        # publishers/subscribers in rospy are automatically run in serperate threads,
        # meaning we dont need to worry about spinning to respond to subscriber callbacks

        while not rospy.is_shutdown():
            self.publishPendingTargets() # see if there's anything to publish
            rospy.sleep(1)  # sleep for one second
                

def main():

    # initialize the node
    rospy.init_node('imaging_handler')

    handler = RosImagingHandler()
    handler.run()

if __name__ == '__main__':
    main()
