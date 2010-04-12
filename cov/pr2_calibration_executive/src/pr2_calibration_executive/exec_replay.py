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

rospy.init_node("capture_executive_node")

# Determine if we want everything to automatically run
autorun = (rospy.myargv()[1] == "auto")
samples_dir = rospy.myargv()[2]
config_dir = rospy.myargv()[3]

executive = pr2_calibration_executive.capture_exec.CaptureExecutive(config_dir)
time.sleep(1.0)

#samples_dir = "/u/vpradeep/ros/pkgs-trunk/wg-ros-pkg/calibration_experimental/pr2_calibration_executive/samples"
#sample_names = ["test_sample_0001", "test_sample_0002"]
#sample_names = ["cam_sample_0001"]

#samples_dir = "/u/vpradeep/dataset_port/samples"
#sample_names = [ "%06u_OldRun7"%n for n in range(1,266) ]

#samples_dir = "/u/vpradeep/ros/pkgs-trunk/wg-ros-pkg/sandbox/trajectory_playback/samples"
#sample_names = [ "r_arm_%04u" % n for n in range(0,15) ]

sample_names = [x for  x in os.listdir(samples_dir) if ".yaml" in x]
sample_names.sort()

print "Going to execute samples: %s" % sample_names

pub = rospy.Publisher("robot_measurement", RobotMeasurement)

ok = True

n = 0
try:
    while ok and not rospy.is_shutdown() and n < len(sample_names):
        sample_name = sample_names[n]
        print "Currently on sample [%s]" % sample_name
        print "(L)oad, (B)ackwards, (F)orwards, (Q)uit"
        if autorun:
            if n == len(sample_names):
                print "Done. Exiting..."
                ok = False
                resp = "Q"
            else:
                resp = "L"
                n += 1   # accidentally skipping first sample... oh well
        else:
            resp = raw_input("> ").upper()
        if resp == "L":
            cur_config = yaml.load(open(samples_dir + "/" + sample_name))
            m_robot = executive.capture(cur_config, rospy.Duration(40))
            if m_robot is None:
                print "************** Didn't get a sample *****************"
            else:
                print "__________ Got a sample! ____________________"
                pub.publish(m_robot)
        elif resp == "B":
            if n is 0:
                print "Already on first sample. Can't do anything"
            else:
                n -= 1
        elif resp == "F":
            if n is len(sample_names)-1:
                print "Already on last sample. Can't do anything"
            else:
                n += 1
        elif resp == "Q":
            print "Exiting..."
            ok = False
        else:
            print "Unknown request"
        print ""
except EOFError:
    print "Exiting"

rospy.spin()
