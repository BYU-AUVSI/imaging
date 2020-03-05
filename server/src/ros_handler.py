#! /usr/bin/env python
import rospy
import cv2
import numpy as np
import os, time # for img saving
from cv_bridge import CvBridge, CvBridgeError # for publishing images saved on server
import cv2
from config import defaultConfigPath, config, createNewBaseImgDir, getLatestBaseImgDir
# database access objects
from dao.incoming_image_dao import IncomingImageDAO
from dao.incoming_gps_dao import IncomingGpsDAO
from dao.incoming_state_dao import IncomingStateDAO
from dao.submitted_target_dao import SubmittedTargetDAO
# ROS messages:
from inertial_sense.msg import GPS
from rosplane_msgs.msg import State
from uav_msgs.msg import CompressedImgWithMeta
# submit image service
import rosservice
from uav_msgs.srv import SubmitImage, SubmitImageResponse
# model classes for the dao
from dao.model.incoming_gps import incoming_gps
from dao.model.incoming_image import incoming_image
from dao.model.incoming_state import incoming_state
from dao.model.submitted_target import submitted_target
import math

# TODO:
from geographiclib.geodesic import Geodesic
#######

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
        print("Startup NEW (Mar 2020) ros imaging handler...")
        currentPath = os.path.dirname(os.path.realpath(__file__))
        self.configPath = rospy.get_param('~config_path', defaultConfigPath())

        self.bridge = CvBridge()

        # gps ingestion setup:
        self.gps_dao_ = IncomingGpsDAO(self.configPath)
        # self.gps_subscriber_ = rospy.Subscriber('/gps', GPS, self.gpsCallback, queue_size=10)
        self.gps_subscriber_ = rospy.Subscriber('/state', State, self.gpsCallback, queue_size=10)
        self.gps_msg_ = incoming_gps()

        # imaging ingestion setup:
        self.img_dao_ = IncomingImageDAO(self.configPath)
        self.img_subscriber_ = rospy.Subscriber("/a6000_ros_node/img/compressed", CompressedImgWithMeta, self.imgCallback,  queue_size = 10)
        self.img_msg_ = incoming_image()
        self.img_msg_.focal_length = 16.0 # this is a safe starting assumption == fully zoomed out on a6000
        self.img_msg_.manual_tap = False
        self.img_msg_.autonomous_tap = False

        # state ingestion setup:
        self.state_dao_ = IncomingStateDAO(self.configPath)
        self.state_subscriber_ = rospy.Subscriber('/state', State, self.stateCallback, queue_size=10)
        self.state_msg_ = incoming_state()
        self.state_interval_ = 0

        basePath = createNewBaseImgDir()
        print("Base dir for images:: {}".format(basePath))

        # service for completed targets. Once a target has gone through the entire
        # system, the server puts the finished target in a table and marks it ready for submission
        self.submit_image_ = None
        print("ROS subscribers are all setup!")

        self.geod = Geodesic.WGS84

    def gpsCallback(self, stateMsg): # originally had 'msg', see lines 52, 92
        """
        Ros subscriber callback. Subscribes to the inertial_sense GPS msg.
        Get the lla values from the GPS message and then pass them to the DAO so 
        they can be inserted into the database
        """
        # if msg.fix_type == GPS.GPS_STATUS_FIX_TYPE_NO_FIX or msg.num_sat == 0:
        #     # dont insert if we dont have a gps fix yet
        #     return
        '''TODO Figure out how to replicate the above ^'''

        '''
        TODO: Evaluate and Update
        self.gps_msg_.time_stamp = msg.header.stamp.to_sec()
        self.gps_msg_.lat  = msg.latitude
        self.gps_msg_.lon  = msg.longitude
        self.gps_msg_.alt  = msg.altitude
        '''
        '''
            Testing - Feb 2020:
            Goal:       Pull GPS data from state, rather than GPS, as state is more accurate due to filtering
            Procedure:  Because the state stores initial coordinates and distance from base in meters, we need to convert
                        from meters to actual GPS points before storing. We do this using a geographiclib 'Direct' function,
                        after estimating the angle between the two points. 

            Notes:      This procedure does result in rounding error. During a simulated test, we estimated this to be an 
                        average error of 0.01% (e.g., 0.1 m off for every 1000 m our plane is from the base station), although
                        it did fluctuate (never surpassing 0.1 %). Overall, we believe the benefits will outweigh the error
        '''
        ### New stuff starts here
        north_dist = stateMsg.position[0]
        east_dist = stateMsg.position[1]
        estimated_azimuth = math.degrees(math.atan2(east_dist, north_dist))
        estimated_total_distance = math.sqrt( (north_dist**2) + (east_dist**2) )
            # Assumes we resolved the state initial lat/lon problem 
        estimated_mav_location = self.geod.Direct(stateMsg.initial_lat, stateMsg.initial_lon, estimated_azimuth, estimated_total_distance, Geodesic.Standard)

        self.gps_msg_.time_stamp = stateMsg.header.stamp.to_sec()
        self.gps_msg_.lat = estimated_mav_location['lat2']
        self.gps_msg_.lon = estimated_mav_location['lon2']
        self.gps_msg_.alt = stateMsg.initial_alt - stateMsg.position[2]
	    ### ... and it ends here


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

    def rotate_about_center(self, src, angle, scale=1.):
        w = src.shape[1]
        h = src.shape[0]
        rangle = np.deg2rad(angle)  # angle in radians
        # now calculate new image width and height
        nw = (abs(np.sin(rangle)*h) + abs(np.cos(rangle)*w))*scale
        nh = (abs(np.cos(rangle)*h) + abs(np.sin(rangle)*w))*scale
        # ask OpenCV for the rotation matrix
        rot_mat = cv2.getRotationMatrix2D((nw*0.5, nh*0.5), angle, scale)
        # calculate the move from the old center to the new center combined
        # with the rotation
        rot_move = np.dot(rot_mat, np.array([(nw-w)*0.5, (nh-h)*0.5,0]))
        # the move only affects the translation, so update the translation
        # part of the transform
        rot_mat[0,2] += rot_move[0]
        rot_mat[1,2] += rot_move[1]
        return cv2.warpAffine(src, rot_mat, (int(np.ceil(nw)), int(np.ceil(nh))))

    def imgCallback(self, msg):
        """
        Ros subscriber callback. Subscribes to the cameras image topic. Saves
        the image file, and passes the corresponding filename and TS to the DAO
        so that it can be inserted into the database
        """
        # create paths for where raw images will be dumped if necessary
        raw_path_ = os.path.join(getLatestBaseImgDir(), 'raw')
        if not os.path.exists(raw_path_):
            os.makedirs(raw_path_)
        # setup file name
        ts = msg.img.header.stamp.to_sec()
        filename = str(ts) + ".jpg"
        fullPath = os.path.join(raw_path_, filename)

        # setup the actual file data
        np_arr = np.fromstring(msg.img.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # if its opencv < 3.0 then use cv2.CV_LOAD_IMAGE_COLOR

        self.img_msg_.time_stamp = ts 
        self.img_msg_.image_path = fullPath
        self.img_msg_.focal_length = msg.focal_length

        orientation = msg.orientation
        # uint8 ORIENTATION_TYPE_ROTATE_180    = 3 # image is rotated 180 degrees
        # uint8 ORIENTATION_TYPE_ROTATE_90_CW  = 6 # image is rotated 90 dec CW
        # uint8 ORIENTATION_TYPE_ROTATE_90_CCW = 8 # image is rotated 90 deg CCW
        if orientation == 3: # Rotated 180
            properlyRotated = self.rotate_about_center(image_np, 180)
        elif orientation == 6: # rotated 90 CW
            properlyRotated = self.rotate_about_center(image_np, 90)
        elif orientation == 8: # rotated 90 CCW
            properlyRotated = self.rotate_about_center(image_np, 270)
        else:
            properlyRotated = image_np

        # write out the image
        cv2.imwrite(fullPath, properlyRotated)

        # insert into the db - returns db id of inserted image
        resultingId = self.img_dao_.addImage(self.img_msg_)
        if resultingId == -1:
            print("FAILED to insert image:")
            print("ts: {}, path: {}, manual_tap: {}, autonomous_tap: {}".format(*self.img_msg_.insertValues()))

    def targetToInteropMsg(self, target):
        """
        Convert a submitted_target model to an InteropImage msg
        returns an InteropImage msg ready for publishing
        """

        img = None
        # figure out the image part of the message
        if hasattr(target, 'crop_path'):
            cv_img = cv2.imread(target.crop_path)
            try:
                img = self.bridge.cv2_to_imgmsg(cv_img, "bgr8")
            except CvBridgeError as e:
                print(e)

        # this dictionary will hold all the values that we care about
        # pushing into the message for whatever target type we're dealing with
        # ie: for emergent it sets irrelevant values to none, to ensure they arent
        # posted to the judges
        targetDict  = target.toAuvsiJson()
        targetDict["image"] = img

        return targetDict

    def submitPendingTargets(self):
        # if there are people actually subscribed to this topic
        if '/imaging/target' not in rosservice.get_service_list():
            # print(Submission service not yet published")
            return

        # if the service exists on the network
        if self.submit_image_ is None:
            # if this is our first time connecting to the image submit service
            self.submit_image_ = rospy.ServiceProxy('/imaging/target', SubmitImage)
        
        target_dao = SubmittedTargetDAO(self.configPath)
        # if there are targets waiting to be submitted to the judges
        if not target_dao.areTargetsPending():
            print("no targets pending")
            return
        
        # then lets submit them
        pending = target_dao.getAllPendingTargets()
        if pending is None or not pending:
            return
        
        for target in pending:
            imageMsg = self.targetToInteropMsg(target)
            resp = self.submit_image_(**imageMsg) # map dictionary into function args for submit_image

            if resp.success:
                # only set a target as submitted if interop says it was successful
                target_dao.setTargetSubmitted(target.target, target.autonomous)

    def run(self):
        # as per (this very old thread): http://ros-users.122217.n3.nabble.com/ros-spinOnce-equivalent-for-rospy-td2317347.html
        # publishers/subscribers in rospy are automatically run in serperate threads,
        # meaning we dont need to worry about spinning to respond to subscriber callbacks

        while not rospy.is_shutdown():
            self.submitPendingTargets() # see if there's anything to publish
            rospy.sleep(1)  # sleep for one second

def main():

    # initialize the node
    rospy.init_node('imaging_handler')

    handler = RosImagingHandler()
    handler.run()

if __name__ == '__main__':
    main()
