#! /usr/bin/env python
import roslib
import rospy
import camera_pose_calibration.msg
import std_msgs.msg
import rosbag
import sys
import os
import rostopic
from rostopic import ROSTopicException

def callback(msg):
    global topic_name, bag_filename
    if (bag_filename):
        bag = rosbag.Bag(bag_filename, 'w')
        bag.write(topic_name, msg)
        print "Saving msg on [%s]" % topic_name
        bag.close()

if len(rospy.myargv()) >= 3:
    msg_type = std_msgs.msg.Empty
    topic_name = rospy.myargv()[1]
    topic_type = rospy.myargv()[2]
    bag_filename = None
    if len(rospy.myargv()) >= 4:
        bag_filename = rospy.myargv()[3]
else:
    #print ""
    print "   Message Saver:"
    print "      Saves the latest message on a topic into a bagfile, and republishes saved message on bootup."
    print "      This is a really convenient way to store state in a system."
    print "   Usage: "
    print "      msg_saver.py [topic] [type] [filename]"
    sys.exit(-1)


print "Topic Name: %s" % topic_name
#pub = rospy.Publisher(topic_name, msg_type, latch=True)

# Figure out the message class, and then create the publisher
try:
    msg_class = roslib.message.get_message_class(topic_type)
except:
    raise ROSTopicException("invalid topic type: %s"%topic_type)
if msg_class is None:
    pkg = roslib.names.resource_name_package(topic_type)
    raise ROSTopicException("invalid message type: %s.\nIf this is a valid message type, perhaps you need to type 'rosmake %s'"%(topic_type, pkg))
# disable /rosout and /rostime as this causes blips in the pubsub network due to rostopic pub often exiting quickly
rospy.init_node("msg_saver", disable_rosout=True, disable_rostime=True)

pub = rospy.Publisher(topic_name, msg_class, latch=True)

# Get the last message on the topic of interest

if bag_filename:
    if (os.path.isfile(bag_filename)):
        print "Found file [%s]" % bag_filename
        bag = rosbag.Bag(bag_filename)
        last_msg = None
        for topic, msg, t in bag.read_messages(topics=[topic_name]):
            last_msg = msg
        if last_msg is not None:
            pub.publish(last_msg)
        else:
            print "Couldn't find any messages on topic [%s]" % topic_name
        bag.close()
    else:
        print "Couldn't find file [%s]. Skipping publishing" % bag_filename

# Listen on the same topic, so that we can update the bag
sub = rospy.Subscriber(topic_name, msg_class, callback)

rospy.spin()
