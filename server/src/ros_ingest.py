import rospy
from dao import DAO
# from inertial_sense.msg import GPS

class RosIngester:
    """
    Subscribes to the given topics and saves their information in 
    a postgresql database
    """

    dao_ = None

    def __init__(self):
        print("Startup ros ingester...")
        self.dao_ = DAO()
        # self.gps_subscriber = rospy.Subscriber('/gps', GPS, self.gpsCallback, queue_size=10)
        # self.raw_subscriber = rospy.Subscriber('/other_camera/image_raw/compressed', compressedImage)
        print("Ingester is all setup!")
        # gps type == inertial_sense/GPS
        # image == sensor_msgs/CompressedImage


    # def gpsCallback(self, msg):
    #     print("heyyyy")

def main():
    # initialize the node
    rospy.init_node('imaging_ingester')

    subscriber = RosIngeter()
    # spin
    try:
        rospy.spin()
    except KeyBoardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()