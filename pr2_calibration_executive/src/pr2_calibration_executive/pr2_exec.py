#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import roslib; roslib.load_manifest('pr2_calibration_launch')
import rospy
import yaml
import sys
import pr2_calibration_executive.capture_exec
import time
from calibration_msgs.msg import RobotMeasurement, CalibrationPattern
import os
import string
from base_control import BaseController
from checkerboard_pose.srv import GetCheckerboardPose
from arm_control import ArmController

in_laser = -1
def laser_check_cb(msg):
    global in_laser
    in_laser = int(msg.success)

print "Starting executive..."
time.sleep(7.0)

rospy.init_node("pr2_capture_executive_node")

samples_dir = rospy.myargv()[1]
config_dir = rospy.myargv()[2]

executive = pr2_calibration_executive.capture_exec.CaptureExecutive(config_dir)
time.sleep(1.0)

left_sample_names  = [x for x in os.listdir(samples_dir + "/left/")  if ".yaml" in x]
right_sample_names = []
far_sample_names = []
right_sample_names = [x for x in os.listdir(samples_dir + "/right/") if ".yaml" in x]
far_sample_names   = [x for x in os.listdir(samples_dir + "/far/")   if ".yaml" in x]

left_sample_names.sort()
right_sample_names.sort()
far_sample_names.sort()

print "Left Samples: \n - %s" % "\n - ".join(left_sample_names)
print "Right Samples: \n - %s" % "\n - ".join(right_sample_names)
print "Far Samples: \n - %s" % "\n - ".join(far_sample_names)

pub = rospy.Publisher("robot_measurement", RobotMeasurement)

far_success_count = 0
left_success_count = 0
left_fail_count = 0
right_success_count = 0
right_fail_count = 0


try:
    # Capture Far Checkerboards
    keep_collecting = True
    full_paths = [samples_dir + "/far/" + x for x in far_sample_names]
    cur_config = yaml.load(open(full_paths[0]))
    m_robot = executive.capture(cur_config, rospy.Duration(0.01))

    auto_mode = True
    print "You are now beginning pr2 calibration."
    print "Note that running calibration in full auto will require a special checkerboard"
    print "More info can be found at http://ros.org/"
    resp = raw_input("Would you like to run on full auto [Y/n]: ")
    if string.upper(resp) == 'N':
	auto_mode = False
    else:
	auto_mode = True
	if not rospy.has_param("/cb_pass_off_config"):
	    print "You are trying to run auto calibration without a cb pass off config file"
	    print "Press <enter> to proceed without auto calibration or 'q' to quit"
	    resp = raw_input("> ")
	    if string.upper(resp) == 'Q':
		sys.exit(1)
	    else:
		auto_mode = False
	else:
	    pass

    print "Please place the large 7x6 checkerboard approx 3m in front of the robot"
    print "in view of the head cameras and tilting laser."
    print "Then press <enter>"
    raw_input("> ")

    laser_check = rospy.Subscriber('/tilt_laser_7x6/laser_checkerboard', CalibrationPattern, laser_check_cb)
    print "Please wait for initial laser scan to complete"
    while not in_laser >= 0:
        pass
 
    print "Press <enter> when ready to collect data"
    resp = raw_input("> ")

    wide_check = rospy.ServiceProxy("wide_get_checkerboard_pose", GetCheckerboardPose)
    narrow_check = rospy.ServiceProxy("narrow_get_checkerboard_pose", GetCheckerboardPose)
    in_wide = wide_check(6, 7, .108, .108)
    in_narrow = narrow_check(6, 7, .108, .108)

    #check that the checkerboards are present
    while True:
	if in_wide and in_narrow and not in_laser:
	    print "the checkerboard has been found in both the narrow and wide stereo cameras, however it is not visible to the laser." 
            print "This is usually due to specular reflection."
	    print "Please move the checkerboard, and press <enter> when ready to try again."
	    raw_input("> ")
	elif not in_wide or not in_narrow:
	    print "The checkerboard is not visible!"
	    print "Please place the large 7x6 checkerboard approx 3m in front of the robot"
    	    print "in view of the head cameras and tilting laser."
    	    print "Press <enter> when ready to try again."
	elif in_wide and in_narrow and in_laser:
	    break

    print "Beginning auto calibration"

    pose = BaseController()
    pose.strafe_to_pose(0, 'wide_get_checkerboard_pose')
    pose.set_axis()     

    if not rospy.is_shutdown() and keep_collecting:
        for cur_sample_path in full_paths:
            success = False
	    x=0
	    while x<3 and not success:
                print "On far sample [%s]" % cur_sample_path
                cur_config = yaml.load(open(cur_sample_path))
		rotation = float(cur_config['rotation'])

		#if the sample does not succeed rotate back toward the previous working pose
		if x>0:
		  if rotation > 0:
		    rotation -= .075
		  if rotation < 0:
		    rotation += .075 
	        print "Current sample rotation:", rotation	
		try:
		    pose.rot_to_pose(rotation, 'wide_get_checkerboard_pose')
		except:
		    rospy.logerr("Error moving robot, aborting auto-calibration")
		    exit()
                
                m_robot = executive.capture(cur_config, rospy.Duration(40))
                if m_robot is None:
                    print "--------------- Failed To Capture a Far Sample -----------------"
		    print "Retrying ......................................................."	
		    x+=1
                else:
                    print "++++++++++++++ Successfully Captured a Far Sample ++++++++++++++"
                    far_success_count += 1
                    pub.publish(m_robot)
		    success = True
                print "Succeeded on %u far samples" % far_success_count

                if rospy.is_shutdown():
                    break
            if not x < 3:
                print "--------------- Failed To Capture a Far Sample -----------------"
                print "Skipping Sample ................................................"


    # Capture Left Arm Data
    if not rospy.is_shutdown():
        pose.rot_to_pose_wh(3)
        full_paths = [samples_dir + "/left/" + x for x in left_sample_names]

        cur_config = yaml.load(open(full_paths[0]))
        m_robot = executive.capture(cur_config, rospy.Duration(0.01))

        if string.upper(resp) == "N":
            print "Skipping left arm samples"
        else:
            for cur_sample_path in full_paths:
                print "On left arm sample [%s]" % cur_sample_path
                cur_config = yaml.load(open(cur_sample_path))
                m_robot = executive.capture(cur_config, rospy.Duration(40))
                if m_robot is None:
                    print "--------------- Failed To Capture a Left Hand Sample -----------------"
                    left_fail_count += 1
                else:
                    print "++++++++++++++ Successfully Captured a Left Hand Sample ++++++++++++++"
                    left_success_count += 1
                    pub.publish(m_robot)
                print "Succeded on %u/%u left arm samples" % (left_success_count, left_fail_count + left_success_count)
                if rospy.is_shutdown():
                    break

    if auto_mode:
        #Pass the checkerboard off to the right gripper
        arm_controller = ArmController(rospy.get_param("/cb_pass_off_config"))
        arm_controller.run()

    elif not auto_mode:
	print "Please move the checkerboard to the robots right gripper."
	print "Press <enter> when you are ready to continue"
	raw_input("> ")

    # Capture Right Arm Data
    if not rospy.is_shutdown():
        full_paths = [samples_dir + "/right/" + x for x in right_sample_names]

        cur_config = yaml.load(open(full_paths[0]))
        m_robot = executive.capture(cur_config, rospy.Duration(0.01))

        if string.upper(resp) == "N":
            print "Skipping right arm samples"
        else:
            for cur_sample_path in full_paths:
                print "On right arm sample [%s]" % cur_sample_path
                cur_config = yaml.load(open(cur_sample_path))
                m_robot = executive.capture(cur_config, rospy.Duration(40))
                if m_robot is None:
                    print "--------------- Failed To Capture a Right Hand Sample -----------------"
                    right_fail_count += 1
                else:
                    print "++++++++++++++ Successfully Captured a Right Hand Sample ++++++++++++++"
                    right_success_count += 1
                    pub.publish(m_robot)
                print "Succeded on %u/%u right arm samples" % (right_success_count, right_fail_count + right_success_count)
                if rospy.is_shutdown():
                    break

except EOFError:
    print "Exiting"

time.sleep(1)

print "Calibration data collection has completed!"
print "Far Samples: %u" % far_success_count
print "Left Arm Samples: %u/%u" % (left_success_count, left_fail_count + left_success_count)
print "Right Arm Samples: %u/%u" % (right_success_count, right_fail_count + right_success_count)
print ""
print "You can now kill this node, along with any other calibration nodes that are running."
