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

from sensor_msgs.msg import CameraInfo

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

def find_board_min(board_pose, corners_x, corners_y, spacing):
    trans = msg_to_pose(board_pose.pose)
    origin = tf.transformations.translation_matrix([-spacing, -spacing, 0.0])
    pose_mat = numpy.dot(trans, origin)
    return pose_to_msg(pose_mat)

def find_board_max(board_pose, corners_x, corners_y, spacing):
    trans = msg_to_pose(board_pose.pose)
    origin = tf.transformations.translation_matrix([((corners_x) * spacing), ((corners_y) * spacing), 0.0])
    pose_mat = numpy.dot(trans, origin)
    return pose_to_msg(pose_mat)

def find_board_center(board_pose, corners_x, corners_y, spacing):
    trans = msg_to_pose(board_pose.pose)
    origin = tf.transformations.translation_matrix([((corners_x - 1) * spacing) / 2.0, ((corners_y - 1) * spacing) / 2.0, 0.0])
    pose_mat = numpy.dot(trans, origin)
    return pose_to_msg(pose_mat)

def project_pose(proj, pose):
    position = pose.pose.position
    point = numpy.array([[position.x], [position.y], [position.z]])
    mult = numpy.dot(proj, point)
    return mult / mult[2]

def depth_calibrator_main(argv=None):
    rospy.init_node(NAME, anonymous=False)
    pose_pub = rospy.Publisher("new_pose", PoseStamped, latch=True)

    corners_x = rospy.get_param('~corners_x', 5)
    corners_y = rospy.get_param('~corners_y', 4)
    spacing = rospy.get_param('~spacing', 0.0245)

    listener = tf.TransformListener()

    print "Waiting to receive camera info for the depth camera..."
    depth_info = rospy.wait_for_message('depth_camera_info', CameraInfo)
    depth_proj = numpy.reshape(numpy.array(depth_info.P), (3,4))[:3, :3]
    depth_frame_id = depth_info.header.frame_id
    print "Got camera info, the depth camera operates in frame %s" % depth_frame_id

    run = True
    while run:

        while(raw_input("Is a checkerboard visible for your image producing camera? (y/n): ") != 'y'):
            print "For the Kinect, you may need to cover the IR projector for the CB to be detected."

        print "OK, thanks! Checking for the 'get_checkerboard_pose' service."

        rospy.wait_for_service('get_checkerboard_pose')

        print "Found the checkerboard service."

        print "Looking for a %d x %d checkerboard, with %.4f spacing...." % (corners_x, corners_y, spacing)

        cb_detector = rospy.ServiceProxy('get_checkerboard_pose', GetCheckerboardPose)
        cb = cb_detector.call(GetCheckerboardPoseRequest(corners_x, corners_y, spacing, spacing))
        img_center = find_board_center(cb.board_pose, corners_x, corners_y, spacing)
        img_msg = PoseStamped()
        img_msg.header = cb.board_pose.header
        img_msg.pose = img_center
        pose_pub.publish(img_msg)

        depth_center = None
        #now we want to transform the pose into the depth frame if the image frame is different
        if depth_frame_id is not None and img_msg.header.frame_id != depth_frame_id:
            img_min = PoseStamped(cb.board_pose.header, find_board_min(cb.board_pose, corners_x, corners_y, spacing))
            img_max = PoseStamped(cb.board_pose.header, find_board_max(cb.board_pose, corners_x, corners_y, spacing))

            print "Waiting for the transform between %s and %s" % (img_msg.header.frame_id, depth_frame_id)
            listener.waitForTransform(depth_frame_id, img_msg.header.frame_id, img_msg.header.stamp, rospy.Duration(2.0))
            depth_pose = listener.transformPose(depth_frame_id, img_msg)
            depth_min = listener.transformPose(depth_frame_id, img_min)
            depth_max = listener.transformPose(depth_frame_id, img_max)

            depth_min_pt = project_pose(depth_proj, depth_min)
            depth_max_pt = project_pose(depth_proj, depth_max)

            cb.min_x = depth_min_pt[0]
            cb.min_y = depth_min_pt[1]
            cb.max_x = depth_max_pt[0]
            cb.max_y = depth_max_pt[1]

            depth_center = depth_pose.pose
        else:
            depth_center = img_center

        print "Successfully found a checkerboard, with depth %.4f, noise_vel: %f, noise_rot: %f in frame %s" % (depth_center.position.z, 
                                                                                                                    cb.noise_vel, cb.noise_rot, 
                                                                                                                    depth_frame_id)


        print "Now we want to look for the center of the checkerboard in the depth image."
        while(raw_input("Have you uncovered the IR projector and turned off any other IR light source? (y/n): ") != 'y'):
            print "Don't be difficult, just do it."

        print "OK, thanks! Waiting for the 'get_checkerboard_center' service."

        rospy.wait_for_service('get_checkerboard_center')

        print "Found the service, looking for the center of the checkerboard bounded by the rectangle (%.2f, %.2f), (%.2f, %.2f)." % (
            cb.min_x, cb.min_y, cb.max_x, cb.max_y)

        center_detector = rospy.ServiceProxy('get_checkerboard_center', GetCheckerboardCenter)
        center_depth = center_detector.call(GetCheckerboardCenterRequest(cb.min_x, cb.max_x, cb.min_y, cb.max_y, depth_center.position.z))

        print "Found the center of the board at depth %.4f in the pointcloud" % (center_depth.depth)

        print "The offset between the img and the point cloud is: %.4f" % (depth_center.position.z - center_depth.depth)

        run = raw_input("Would you like to run again? (y/n): ") == 'y'

if __name__ == '__main__':
    depth_calibrator_main()
