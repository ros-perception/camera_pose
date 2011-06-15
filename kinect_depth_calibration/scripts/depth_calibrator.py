#! /usr/bin/python
#***********************************************************
#* Software License Agreement (BSD License)
#*
#*  Copyright (c) 2009, Willow Garage, Inc.
#*  All rights reserved.
#*
#*  Redistribution and use in source and binary forms, with or without
#*  modification, are permitted provided that the following conditions
#*  are met:
#*
#*   * Redistributions of source code must retain the above copyright
#*     notice, this list of conditions and the following disclaimer.
#*   * Redistributions in binary form must reproduce the above
#*     copyright notice, this list of conditions and the following
#*     disclaimer in the documentation and/or other materials provided
#*     with the distribution.
#*   * Neither the name of Willow Garage, Inc. nor the names of its
#*     contributors may be used to endorse or promote products derived
#*     from this software without specific prior written permission.
#*
#*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#*  POSSIBILITY OF SUCH DAMAGE.
#* 
#* Author: Eitan Marder-Eppstein
#***********************************************************
PKG = 'kinect_depth_calibration'
NAME = 'depth_calibrator'

import roslib; roslib.load_manifest(PKG)

import rospy
from geometry_msgs.msg import PoseStamped
from kinect_depth_calibration.srv import GetCheckerboardPose, GetCheckerboardPoseResponse, GetCheckerboardPoseRequest

def depth_calibrator_main(argv=None):
    rospy.init_node(NAME, anonymous=False)
    while(raw_input("Have you covered the projector on the Kinect? (y/n): ") != 'y'):
        print "You should cover the projector on the Kinect"

    while(raw_input("Have you placed a checkerboard in front of the Kinect and provided a source of IR illumination? (y/n): ") != 'y'):
        print "Go find a checkerboard and an IR light source."

    print "OK, thanks! Checking for the 'get_checkerboard_pose' service."
    rospy.wait_for_service('get_checkerboard_pose')
    print "Found the checkerboard service."
    x_corners = int(raw_input("How many x corners? : "))
    y_corners = int(raw_input("How many y corners? : "))
    spacing = float(raw_input("What is the spacing of the checkerboard? : "))
    print "Looking for a %d x %d checkerboard, with %.4f spacing...." % (x_corners, y_corners, spacing)
    cb_detector = rospy.ServiceProxy('get_checkerboard_pose', GetCheckerboardPose)
    cb = cb_detector.call(GetCheckerboardPoseRequest(x_corners, y_corners, spacing, spacing))
    print "Successfully found a checkerboard."
    print cb



if __name__ == '__main__':
    depth_calibrator_main()
