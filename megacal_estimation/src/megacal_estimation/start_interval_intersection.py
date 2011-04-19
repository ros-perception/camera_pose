#! /usr/bin/env python

import roslib; roslib.load_manifest('megacal_estimation')
import rospy
import actionlib

from interval_intersection.msg import *

if __name__ == '__main__':
    rospy.init_node('start_interval_intersection')
    client = actionlib.SimpleActionClient('interval_intersection_config', ConfigAction)
    print "Waiting for Server"
    client.wait_for_server()
    print "Found Server"

    goal = ConfigGoal()
    # Fill in the goal here
    goal.topics = ['kinect_a/settled_interval', 'kinect_b/settled_interval']
    client.send_goal_and_wait(goal)
    print "Done sending goal"
