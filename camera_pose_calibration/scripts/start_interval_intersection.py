#! /usr/bin/env python

import rospy
import actionlib
import sys

from interval_intersection.msg import *

if __name__ == '__main__':
    # get list of camera's from arguments
    args = rospy.myargv(sys.argv)
    if len(args) < 2:
        rospy.logfatal("Usage: start_interval_intersection ns_1 [ns_2] [ns_3]")
        raise


    rospy.init_node('start_interval_intersection')
    client = actionlib.SimpleActionClient('interval_intersection_config', ConfigAction) # ConfigAction, message in pr2_calibration stack, interval_intersection pkg
    print "Waiting for Server"
    client.wait_for_server()
    print "Found Server"

    # Fill in the goal here
    goal = ConfigGoal()
    goal.topics = [ a+'/settled_interval' for a in args[1:]]
    client.send_goal_and_wait(goal)
    print "Done sending goal"
