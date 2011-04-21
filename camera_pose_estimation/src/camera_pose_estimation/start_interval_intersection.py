#! /usr/bin/env python

import roslib; roslib.load_manifest('camera_pose_estimation')
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
    goal.topics = ['camera_a/settled_interval', 'camera_b/settled_interval']
    client.send_goal_and_wait(goal)
    print "Done sending goal"
