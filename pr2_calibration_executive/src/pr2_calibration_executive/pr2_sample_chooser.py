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

far_success_count = 0
left_success_count = 0
left_fail_count = 0
right_success_count = 0
right_fail_count = 0


try:
    # Capture Far Checkerboards
    keep_collecting = True

    while not rospy.is_shutdown() and keep_collecting:
        print "Choose L,R, or F"
        resp = raw_input("> ")
        if string.upper(resp) == "L":
            print "Choose sample index"
            resp = raw_input("> ")
            cur_sample_path = samples_dir + "/left/" + left_sample_names[int(resp)]
        elif string.upper(resp) == "R":
            print "Choose sample index"
            resp = raw_input("> ")
            cur_sample_path = samples_dir + "/right/" + right_sample_names[int(resp)]
        elif string.upper(resp) == "F":
            cur_sample_path = samples_dir + "/far/" + far_sample_names[0]
        elif string.upper(resp) == "":
            print "Repeating Previous Command"
        else:
            print "Unknown input. Exiting"
            break

        print "On sample [%s]" % cur_sample_path
        cur_config = yaml.load(open(cur_sample_path))

        print "Choose Timeout"
        timeout = raw_input("> ")
        if timeout == "":
            timeout = "1.0"

        m_robot = executive.capture(cur_config, rospy.Duration(float(timeout)))
        if m_robot is None:
            print "--------------- Failed To Capture a Far Sample -----------------"
        else:
            print "++++++++++++++ Successfully Captured a Far Sample ++++++++++++++"
            pub.publish(m_robot)


except EOFError:
    print "Exiting"

