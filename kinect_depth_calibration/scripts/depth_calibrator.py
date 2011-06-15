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
import tf
import numpy
import math
from geometry_msgs.msg import PoseStamped
from kinect_depth_calibration.srv import GetCheckerboardPose, GetCheckerboardPoseResponse, GetCheckerboardPoseRequest
from kinect_depth_calibration.srv import GetCheckerboardCenter, GetCheckerboardCenterResponse, GetCheckerboardCenterRequest

from geometry_msgs.msg import Quaternion, Pose

def msg_to_quaternion(msg):
  return [msg.x, msg.y, msg.z, msg.w]

def quaternion_to_msg(q):
  msg = Quaternion()
  msg.x = q[0]
  msg.y = q[1]
  msg.z = q[2]
  msg.w = q[3]
  return msg

def msg_to_pose(msg):
  trans = tf.transformations.quaternion_matrix(msg_to_quaternion(msg.orientation))
  trans[0, 3] = msg.position.x
  trans[1, 3] = msg.position.y
  trans[2, 3] = msg.position.z
  trans[3, 3] = 1.0
  return trans

def pose_to_msg(pose):
  msg = Pose()
  msg.position.x = pose[0, 3]
  msg.position.y = pose[1, 3]
  msg.position.z = pose[2, 3]
  q = tf.transformations.quaternion_from_matrix(pose)
  msg.orientation.x = q[0]
  msg.orientation.y = q[1]
  msg.orientation.z = q[2]
  msg.orientation.w = q[3]
  return msg

def find_board_center(board_pose, corners_x, corners_y, spacing):
    trans = msg_to_pose(board_pose.pose)
    origin = tf.transformations.translation_matrix([((corners_x - 1) * spacing) / 2.0, ((corners_y - 1) * spacing) / 2.0, 0.0])
    pose_mat = numpy.dot(trans, origin)
    return pose_to_msg(pose_mat)

def depth_calibrator_main(argv=None):
    rospy.init_node(NAME, anonymous=False)
    pose_pub = rospy.Publisher("new_pose", PoseStamped, latch=True)

    corners_x = rospy.get_param('corners_x', 5)
    corners_y = rospy.get_param('corners_y', 4)
    spacing = rospy.get_param('spacing', 0.0245)

    run = True
    while run:

        while(raw_input("Have you covered the projector on the Kinect? (y/n): ") != 'y'):
            print "You should cover the projector on the Kinect"

        while(raw_input("Have you placed a checkerboard in front of the Kinect and provided a source of IR illumination? (y/n): ") != 'y'):
            print "Go find a checkerboard and an IR light source."

        print "OK, thanks! Checking for the 'get_checkerboard_pose' service."

        rospy.wait_for_service('get_checkerboard_pose')

        print "Found the checkerboard service."

        print "Looking for a %d x %d checkerboard, with %.4f spacing...." % (corners_x, corners_y, spacing)

        cb_detector = rospy.ServiceProxy('get_checkerboard_pose', GetCheckerboardPose)
        cb = cb_detector.call(GetCheckerboardPoseRequest(corners_x, corners_y, spacing, spacing))
        ir_center = find_board_center(cb.board_pose, corners_x, corners_y, spacing)
        ir_msg = PoseStamped()
        ir_msg.header = cb.board_pose.header
        ir_msg.pose = ir_center
        pose_pub.publish(ir_msg)

        print "Successfully found a checkerboard, with depth %.4f" % (ir_center.position.z)

        print "Now we want to look for the center of the checkerboard in the depth image."
        while(raw_input("Have you uncovered the IR projector and turned off any other IR light source? (y/n): ") != 'y'):
            print "Don't be difficult, just do it."

        print "OK, thanks! Waiting for the 'get_checkerboard_center' service."

        rospy.wait_for_service('get_checkerboard_center')

        print "Found the service, looking for the center of the checkerboard bounded by the rectangle (%.2f, %.2f), (%.2f, %.2f)." % (
            cb.min_x, cb.min_y, cb.max_x, cb.max_y)

        center_detector = rospy.ServiceProxy('get_checkerboard_center', GetCheckerboardCenter)
        center_depth = center_detector.call(GetCheckerboardCenterRequest(cb.min_x, cb.max_x, cb.min_y, cb.max_y, ir_center.position.z))

        print "Found the center of the board at depth %.4f in the pointcloud" % (center_depth.depth)

        print "The offset between the ir and the point cloud is: %.4f" % (ir_center.position.z - center_depth.depth)

        run = raw_input("Would you like to run again? (y/n): ") == 'y'

if __name__ == '__main__':
    depth_calibrator_main()
