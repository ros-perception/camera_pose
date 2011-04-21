#! /usr/bin/env python

import roslib; roslib.load_manifest('camera_pose_estimation')
import rospy
import actionlib
import sys

from image_cb_detector.msg import *

if __name__ == '__main__':
    rospy.init_node('start_stereo_detector')

    print "Configuring checkerboard size %sx%s, checker size %s"%(sys.argv[1], sys.argv[2], sys.argv[3])

    client = actionlib.SimpleActionClient('cb_detector_config', ConfigAction)
    print "Waiting for Server"
    client.wait_for_server()
    print "Found Server"

    goal = ConfigGoal()
    # Fill in the goal here
    goal.num_x = int(sys.argv[1])
    goal.num_y = int(sys.argv[2])
    goal.spacing_x = float(sys.argv[3])
    goal.spacing_y = float(sys.argv[3])
    goal.width_scaling = 1.0
    goal.height_scaling = 1.0
    goal.subpixel_window = 5
    goal.subpixel_zero_zone = 1
    client.send_goal_and_wait(goal)
    print "Done sending goal"
