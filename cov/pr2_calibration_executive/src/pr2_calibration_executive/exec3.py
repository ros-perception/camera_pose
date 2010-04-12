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

rospy.init_node("capture_executive_node")
executive = pr2_calibration_executive.capture_exec.CaptureExecutive()
time.sleep(1.0)


samples_dir = "/u/vpradeep/ros/pkgs-trunk/wg-ros-pkg/calibration_experimental/pr2_calibration_executive/samples"
#sample_names = ["test_sample_0001", "test_sample_0002"]
sample_names = ["cam_sample_0001"]

#samples_dir = "/u/vpradeep/dataset_port/samples"
#sample_names = [ "%06u_OldRun7"%n for n in range(1,266) ]

pub = rospy.Publisher("robot_measurement", RobotMeasurement)

for sample_name in sample_names:
    if rospy.is_shutdown():
        break

    print "Running sample [%s]" % sample_name
    cur_config = yaml.load(open(samples_dir + "/" + sample_name + ".yaml"))
    m_robot = executive.capture(cur_config, rospy.Duration(10))
    if m_robot is None:
        print "************** Didn't get a sample *****************"
    else:
        print "__________ Got a sample! ____________________"
        pub.publish(m_robot)

rospy.spin()
