#! /usr/bin/env python

import rospy
import actionlib

from monocam_settler.msg import *

if __name__ == '__main__':
    rospy.init_node('start_monocam_settler')
    client = actionlib.SimpleActionClient('monocam_settler_config', ConfigAction)
    print "Waiting for Server"
    client.wait_for_server()
    print "Found Server"

    goal = ConfigGoal()
    # Fill in the goal here
    goal.tolerance = 1.0;
    goal.ignore_failures = 1;
    goal.max_step = rospy.Duration(15.0)
    goal.cache_size = 100;
    client.send_goal_and_wait(goal)
    print "Done sending goal"
