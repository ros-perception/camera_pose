#! /usr/bin/env python

# yliu 
# Aug 5, 2011 International Beer Day


import roslib;roslib.load_manifest('camera_pose_toolkits')

import rospy
import camera_pose_calibration.msg
import std_msgs.msg
import rosbag
import sys
import os
import rostopic
import time
from geometry_msgs.msg import TransformStamped

def callback(msg):
    global topic_name, bag_filename, best_list
    if (bag_filename):
        if len(best_list) < 1:
            best_list.append(msg)
        else:
            for b in best_list:
                # print msg.header.frame_id
                # print msg.child_frame_id
                # print b.header.frame_id
                # print b.child_frame_id
                # print (msg.header.frame_id == b.header.frame_id and msg.child_frame_id == b.child_frame_id)
                if msg.header.frame_id == b.header.frame_id and msg.child_frame_id == b.child_frame_id:
                    print 'repeat' 
                    best_list[best_list.index(b)] = msg
                    break
            else:
                print 'append'
                best_list.append(msg)
        print best_list

        bag = rosbag.Bag(bag_filename, 'w')  #overwrite
        for b in best_list:
            bag.write(topic_name, b)
        
        print "Saving msg on [%s]" % topic_name
        print "number of messages in bag: %d" % len(best_list)
        bag.close()

                       



if len(rospy.myargv()) >= 1:
    bag_filename = rospy.myargv()[1]
else:
    print "   Usage: "
    print "      transform_msg_saver.py [bagfilename]"
    sys.exit(-1)
    
topic_name = 'camera_pose_static_transform_update'


rospy.init_node("transform_msg_saver")


pub = rospy.Publisher(topic_name, TransformStamped, latch=True)

# on boot up

best_list = []

if bag_filename:
    if (os.path.isfile(bag_filename)):
        print "Found file [%s]" % bag_filename
        bag = rosbag.Bag(bag_filename)
        for topic, msg, t in bag.read_messages(topics=[topic_name]):
            best_list.append (msg)
        else:
            print "Couldn't find any messages on topic [%s]" % topic_name
        bag.close()

        count = 0
        
        while pub.get_num_connections() == 0 and count < 700 :  # get number of connections to other ROS nodes for this topic
            time.sleep(0.01)
            count += 1
      
        if count == 700 :
            print 'trasform_message_saver :'
            print 'Waiting for subscriber(s) to topic [camera_pose_static_transform_update] has timed out.'
            print 'The on-start publishing starts without further waiting.'
            print 'If your bag file [%s] contains multiple previously saved messages, only the latest message is likely to be received.' % bag_filename
            print 'Just a heads-up.'
   
        for b in best_list:
            pub.publish(b)
            time.sleep(0.01)
    else:
        print "Couldn't find file [%s]. Skipping on-start publishing" % bag_filename

# Listen on the same topic, so that we can update the bag
sub = rospy.Subscriber(topic_name, TransformStamped, callback)

rospy.spin()
