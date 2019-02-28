import rosbag
import argparse
from ros_handler import RosImagingHandler


def parse():
    # parse arguments. Require user to specify a bag name
    parser = argparse.ArgumentParser(description="Imaging Rosbag Ingester")
    parser.add_argument("bag_name")
    args = parser.parse_args()
    print("Attempting to parse {}".format(args.bag_name))

    bag = rosbag.Bag(args.bag_name)
    handler = RosImagingHandler() # setup the handler

    print("Ready? Begin!")
    # read in all the messages we care about
    for topic, msg, t in bag.read_messages(topics=['/a6000_ros_node/img', '/gps', '/state', '/a6000_ros_node/img/focal_length']):
        if topic == '/a6000_ros_node/img':
            handler.imgCallback(msg)
        elif topic == '/gps':
            handler.gpsCallback(msg)
        elif topic == '/state':
            handler.stateCallback(msg)
        elif topic == '/a6000_ros_node/img/focal_length':
            handler.flCallback(msg)

    print("DONE!")
    bag.close()

if __name__ == "__main__":
    parse()