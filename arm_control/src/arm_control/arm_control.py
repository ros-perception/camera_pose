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
# Author: Ze'ev Klapow

import roslib; roslib.load_manifest("pr2_calibration_launch")
import rospy
import actionlib
import yaml

from trajectory_msgs.msg import *
from geometry_msgs.msg import *
from pr2_controllers_msgs.msg import *

class Arm(yaml.YAMLObject):
	yaml_tag = u'!Arm'
	def __setstate__(self, state):
		self.__dict__ = state
		self.arm  = self.arm[0]
		#Create a goal message for this arm movement
		goal = JointTrajectoryGoal()
		goal.trajectory.joint_names = ['%s_shoulder_pan_joint' % self.arm, '%s_shoulder_lift_joint' % self.arm, '%s_upper_arm_roll_joint'%self.arm, '%s_elbow_flex_joint' % self.arm, '%s_forearm_roll_joint' % self.arm, '%s_wrist_flex_joint' % self.arm, '%s_wrist_roll_joint' % self.arm]
		for p, x in enumerate(self.waypoints):
			goal.trajectory.points.append(JointTrajectoryPoint())
			goal.trajectory.points[p].positions = x
			if p == 0:
				goal.trajectory.points[p].time_from_start = rospy.Duration(self.speeds[p])
			else:
				goal.trajectory.points[p].time_from_start = goal.trajectory.points[p-1].time_from_start + rospy.Duration(self.speeds[p])
		self.goal = goal
		self.client = None		
	
	def set_client(self, clients):
		self.client = clients[self.arm]

	def send(self):
		if self.client is None:
			raise
		self.client.send_goal(self.goal)
		self.client.wait_for_result()

class Grip(yaml.YAMLObject):
	yaml_tag = u'!Grip'
	def __setstate__(self, state):
		self.__dict__ = state
		self.gripper = self.gripper[0]
		goal = Pr2GripperCommandGoal()
		goal.command.position = self.pose
		goal.command.max_effort = self.effort
		self.goal = goal
		self.client = None

	def set_client(self, clients):
		self.client = clients[self.gripper]	

	def send(self):
		if self.client is None:
			raise
		self.client.send_goal(self.goal)
		self.client.wait_for_result()

class Config(yaml.YAMLObject):
	yaml_tag = u'!Config'
	def __setstate__(self, state):
		self.__dict__ = state
		self.r_arm_client = actionlib.SimpleActionClient(self.r_arm_topic, JointTrajectoryAction)
		self.l_arm_client = actionlib.SimpleActionClient(self.l_arm_topic, JointTrajectoryAction)
		self.r_arm_client.wait_for_server()
		self.l_arm_client.wait_for_server()
		self.arms = {'r': self.r_arm_client, 'l': self.l_arm_client}

		self.r_grip_client = actionlib.SimpleActionClient(self.r_grip_topic, Pr2GripperCommandAction)
		self.l_grip_client = actionlib.SimpleActionClient(self.l_grip_topic, Pr2GripperCommandAction)
		self.r_grip_client.wait_for_server()
		self.l_grip_client.wait_for_server()	
		self.grips = {'r': self.r_grip_client, 'l': self.l_grip_client}
		
	def set_clients(self, objs):
		for x in objs:
			if isinstance(x, Arm):
				x.set_client(self.arms)
			elif isinstance(x, Grip):
				x.set_client(self.grips)

class ArmController():
	def __init__(self, file):
		data = yaml.load(open(file))
		self.config = data[0]
		self.actions = data[1:]
		self.config.set_clients(self.actions)
		
	def run(self):
		for action in self.actions:
			action.send()
	

if __name__ == "__main__":
	rospy.init_node("arm_control")
	controller = ArmController("pass_off.yaml")
	controller.run()
