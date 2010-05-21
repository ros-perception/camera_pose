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
from calibration_msgs.msg import RobotMeasurement
import os
import string

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

try:
    # Capture Far Checkerboards
    keep_collecting = True
    while not rospy.is_shutdown() and keep_collecting:
        full_paths = [samples_dir + "/far/" + x for x in far_sample_names]

        cur_config = yaml.load(open(full_paths[0]))
        m_robot = executive.capture(cur_config, rospy.Duration(0.01))

        print "Please place the large 6x8 checkerboard approx 3m in front of the robot"
        print "in view of the head cameras and tilting laser."
        print "Press <enter> when ready to collect data, or type \"N\" if done collecting large checkerboards"
        resp = raw_input("> ")
        if string.upper(resp) == "N":
            print "Done collecting far samples"
            keep_collecting = False
        else:
            for cur_sample_path in full_paths:
                print "On right arm sample [%s]" % cur_sample_path
                cur_config = yaml.load(open(cur_sample_path))
                m_robot = executive.capture(cur_config, rospy.Duration(40))
                if m_robot is None:
                    print "************** Didn't get a sample *****************"
                else:
                    print "__________ Got a sample! ____________________"
                    pub.publish(m_robot)
                if rospy.is_shutdown():
                    break

    # Capture Left Arm Data
    if not rospy.is_shutdown():
        full_paths = [samples_dir + "/left/" + x for x in left_sample_names]

        cur_config = yaml.load(open(full_paths[0]))
        m_robot = executive.capture(cur_config, rospy.Duration(0.01))

        print "Please put the checkerboard in the left hand (open/close the gripper with the joystick's left/right D-Pad buttons). press <enter> to continue.  Type N to skip"
        resp = raw_input("press <enter> ")
        if string.upper(resp) == "N":
            print "Skipping left arm samples"
        else:
            for cur_sample_path in full_paths:
                print "On left arm sample [%s]" % cur_sample_path
                cur_config = yaml.load(open(cur_sample_path))
                m_robot = executive.capture(cur_config, rospy.Duration(40))
                if m_robot is None:
                    print "************** Didn't get a sample *****************"
                else:
                    print "__________ Got a sample! ____________________"
                    pub.publish(m_robot)
                if rospy.is_shutdown():
                    break

    # Capture Right Arm Data
    if not rospy.is_shutdown():
        full_paths = [samples_dir + "/right/" + x for x in right_sample_names]

        cur_config = yaml.load(open(full_paths[0]))
        m_robot = executive.capture(cur_config, rospy.Duration(0.01))

        print "Please put the checkerboard in the right hand (open/close the gripper with the joystick's square/circle buttons). press <enter> to continue..."
        resp = raw_input("press <enter> ")
        if string.upper(resp) == "N":
            print "Skipping right arm samples"
        else:
            for cur_sample_path in full_paths:
                print "On right arm sample [%s]" % cur_sample_path
                cur_config = yaml.load(open(cur_sample_path))
                m_robot = executive.capture(cur_config, rospy.Duration(40))
                if m_robot is None:
                    print "************** Didn't get a sample *****************"
                else:
                    print "__________ Got a sample! ____________________"
                    pub.publish(m_robot)
                if rospy.is_shutdown():
                    break

except EOFError:
    print "Exiting"

time.sleep(1)

print "Calibration data collection has completed! You can now kill this node, along with any other calibration nodes that are running."
