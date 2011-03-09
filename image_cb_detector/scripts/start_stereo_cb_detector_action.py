#! /usr/bin/env python

import roslib; roslib.load_manifest('image_cb_detector')
import rospy
import actionlib

from image_cb_detector.msg import *

if __name__ == '__main__':
    rospy.init_node('start_stereo_detector')
    client = actionlib.SimpleActionClient('cb_detector_config', ConfigAction)
    print "Waiting for Server"
    client.wait_for_server()
    print "Found Server"

    goal = ConfigGoal()
    # Fill in the goal here
    goal.num_x = 6
    goal.num_y = 7
    goal.spacing_x = 0.108
    goal.spacing_y = 0.108
    goal.width_scaling = 1.0
    goal.height_scaling = 1.0
    goal.subpixel_window = 5
    goal.subpixel_zero_zone = 1
    client.send_goal_and_wait(goal)
    print "Done sending goal"
