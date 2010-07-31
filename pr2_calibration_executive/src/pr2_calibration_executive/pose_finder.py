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
import tf
import numpy
import math
import actionlib

from move_base_msgs.msg import *
from geometry_msgs.msg import *
from checkerboard_pose.srv import GetCheckerboardPose
from pr2_controllers_msgs.msg import PointHeadAction, PointHeadGoal
from move_base_msgs.msg import MoveBaseAction

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

class PoseFinder:
  def __init__(self):
    self.listener = tf.TransformListener()

    self.client = actionlib.SimpleActionClient('pose_base_controller', MoveBaseAction)
    self.head = actionlib.SimpleActionClient('head_traj_controller/point_head_action', PointHeadAction)
    rospy.loginfo("Waiting for move_base action server to come up")
    self.client.wait_for_server()
    self.head.wait_for_server()
    rospy.loginfo("Action server is up!")

    self.last_rot = 0
    self.set_axis()

  def set_axis(self):
    robot_pose = PoseStamped()
    robot_pose.header.frame_id = 'base_link'
    robot_pose.header.stamp= rospy.Time.now()

    self.listener.waitForTransform('odom_combined', robot_pose.header.frame_id, robot_pose.header.stamp, rospy.Duration(2.0))
    self.start_pose = self.listener.transformPose('odom_combined', robot_pose)

  def strafe_to_pose(self, off, serv):
    serv = rospy.ServiceProxy(serv, GetCheckerboardPose)
    cb_pose = None
    goto = None

    try:
      cb_pose = serv(6, 7, .108, .108)
    except:
      rospy.logwarn("Could not get checkerboard pose")
      raise
    else:
      goto = self.get_strafe_pose(cb_pose.board_pose, off)
      goal = MoveBaseGoal()
      goal.target_pose = goto
      self.client.send_goal(goal)
      rospy.loginfo("Goal sent to robot, waiting for response")
      self.client.wait_for_result()

  def rot_to_pose_wh(self, rot):
    dest = self.get_rot_pose(rot)

    goal = MoveBaseGoal()
    goal.target_pose = dest
    self.client.send_goal(goal)
    rospy.loginfo("Goal sent to robot, waiting for response")
    self.client.wait_for_result()

  def rot_to_pose(self, rot, serv):
    serv = rospy.ServiceProxy(serv, GetCheckerboardPose)
    try:
      corner_pose = serv(6, 7, .108, .108).board_pose
    except:
      rospy.logwarn("Could not get checkerboard pose")
      raise

    trans = msg_to_pose(corner_pose.pose)
    origin = tf.transformations.translation_matrix([(7 * .108) / 2, (6 * .108) / 2, 0])
    pose_mat =  numpy.dot(trans, origin)
    corner_pose.pose = pose_to_msg(pose_mat)

    #get the checkerboard pose in the odom frame
    self.listener.waitForTransform('odom_combined', corner_pose.header.frame_id, corner_pose.header.stamp, rospy.Duration(2.0))
    cb_pose = self.listener.transformPose('odom_combined', corner_pose)

    #put into a goal message that we will send to the robot after it has rotated
    head_goal = PointHeadGoal()
    head_goal.target.header.frame_id = cb_pose.header.frame_id
    head_goal.target.point.x = cb_pose.pose.position.x
    head_goal.target.point.y = cb_pose.pose.position.y
    head_goal.target.point.z = cb_pose.pose.position.z

    head_goal.pointing_frame = "high_def_frame"
    head_goal.pointing_axis.x = 1
    head_goal.pointing_axis.y = head_goal.pointing_axis.z =0 
  
    dest = self.get_rot_pose(rot)

    goal = MoveBaseGoal()
    goal.target_pose = dest
    self.client.send_goal(goal)
    rospy.loginfo("Goal sent to robot, waiting for response")
    self.client.wait_for_result() 

    self.head.send_goal(head_goal)
    rospy.loginfo("Moving head")
    self.head.wait_for_result()

  def get_rot_pose(self, rot):
    odom_pose = self.start_pose

    final_rot = rot - self.last_rot

    rot_q = tf.transformations.quaternion_from_euler(0.0, 0.0, final_rot)
    new_q = tf.transformations.quaternion_multiply(rot_q, msg_to_quaternion(self.start_pose.pose.orientation))

    self.last_rot = rot

    #we want to remove any pitch and roll from the goal... so we'll just use the yaw element
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(new_q)

    #now, we're ready to construct the quaternion for what we'll send to navigation
    odom_pose.pose.orientation = quaternion_to_msg(tf.transformations.quaternion_from_euler(0.0, 0.0, yaw))
    return odom_pose

  def get_strafe_pose(self, opt_pose, dist_from):
    trans = msg_to_pose(opt_pose.pose)
    origin = tf.transformations.translation_matrix([(7 * .108) / 2, 0, 0])
    pose_mat =  numpy.dot(trans, origin)
    opt_pose.pose = pose_to_msg(pose_mat)

    #rospy.loginfo("Recieved checkerboard pose in optical frame:\n %s", opt_pose)
    self.listener.waitForTransform('odom_combined', opt_pose.header.frame_id, opt_pose.header.stamp, rospy.Duration(2.0))
    board_pose = self.listener.transformPose('base_link', opt_pose)

    #rospy.loginfo("Transformed opt_pose into base_link frame:\n %s", board_pose)

    dest_pose = PoseStamped()
    dest_pose.header.stamp = board_pose.header.stamp
    dest_pose.header.frame_id = board_pose.header.frame_id
    dest_pose.pose.position.y = board_pose.pose.position.y + dist_from
    dest_pose.pose.position.x = dest_pose.pose.position.z =0
    dest_pose.pose.orientation.x = dest_pose.pose.orientation.y = dest_pose.pose.orientation.z = 0.0
    dest_pose.pose.orientation.w = 1.0

    #rospy.loginfo("Created destination pose in base_link frame:\n %s", dest_pose)

    return dest_pose

if __name__ == "__main__":
  rospy.init_node("strafe_pose_test")

  pose = PoseFinder()
  while True:
    pose.rot_to_pose(1, 'wide_get_checkerboard_pose')
    rospy.sleep(10)
    pose.rot_to_pose(0, 'wide_get_checkerboard_pose')
    rospy.sleep(10)
    pose.rot_to_pose(-1, 'wide_get_checkerboard_pose')
    rospy.sleep(10)
  rospy.spin()
