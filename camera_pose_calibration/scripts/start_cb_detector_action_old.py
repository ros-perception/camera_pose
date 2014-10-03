#! /usr/bin/env python

import rospy
import actionlib
import sys

from image_cb_detector.msg import *


def usage():
    print "Usage:"
    print "   start_cb_detector_action_old.py num_x num_y spacing [image_scaling]"


if __name__ == '__main__':
    rospy.init_node('start_checkerboard_detector')

    if len(rospy.myargv()) < 4:
        usage()
        sys.exit(-1)
    elif len(rospy.myargv()) < 5:
        scaling = 1.0
    else:
        scaling = rospy.myargv()[4]

    print "Configuring checkerboard size %sx%s, checker size %s"%(rospy.myargv()[1], rospy.myargv()[2], rospy.myargv()[3])

    client = actionlib.SimpleActionClient('cb_detector_config', ConfigAction)
    print "Waiting for Server"
    client.wait_for_server()
    print "Found Server"

    goal = ConfigGoal()
    # Fill in the goal here
    goal.num_x = int(rospy.myargv()[1])
    goal.num_y = int(rospy.myargv()[2])
    goal.spacing_x = float(rospy.myargv()[3])
    goal.spacing_y = float(rospy.myargv()[3])
    goal.width_scaling = scaling
    goal.height_scaling = scaling
    goal.subpixel_window = 5
    goal.subpixel_zero_zone = 1
    client.send_goal_and_wait(goal)
    print "Done sending goal"
