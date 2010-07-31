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
	def __init__(self, arm, waypoints, speeds):
		self.arm  = arm[0]
		#Create a goal message for this arm movement
		goal = JointTrajectoryGoal()
		for p, x in enumerate(waypoints):
			goal.trajectory.points.append(JointTrajectoryPoint())
			goal.trajectory.points[p].positions = x
			if p == 0:
				goal.trajectory.points[p].time_from_start = speeds[p]
			else:
				goal.trajectory.points[p].time_from_start = goal.trajectory.points[p-1] + speeds[p]
		self.goal = goal		
	
	def set_client(self, clients):
		self.client = clients[self.arm]

class Grip(yaml.YAMLObject):
	yaml_tag = u'!Grip'
	def __init__(self, gripper, pose, effort):
		self.gripper = gripper[0]
		goal = Pr2GripperCommandGoal()
		goal.command.position = pose
		goal.command.max_effort = effort
		

class Config(yaml.YAMLObject):
	yaml_tag = u'!Config'
	def __init__(self, r_arm_topic, l_arm_topic, r_grip_topic, l_grip_topic):
		r_arm_client = actionlib.SimpleActionClient(r_arm_topic, JointTrajectoryAction)
		l_arm_client = actionlib.SimpleActionClient(l_arm_topic, JointTrajectoryAction)
		r_arm_client.wait_for_server()
		l_arm_client.wait_for_server()
		
		self.arms = {'r': r_arm_client, 'l': l_arm_client}
		
		r_grip_client = actionlib.SimpleActionClient(r_grip_topic, Pr2GripperCommandAction)
		l_grip_client = actionlib.SimpleActionClient(l_grip_topic, Pr2GripperCommandAction)
		r_grip_client.wait_for_server()
		l_grip_client.wait_for_server()
		
		self.grips = {'r': r_grip_client, 'l': l_grip_client}
		
	def set_clients(self, objs):
		for x in objs:
			if isinstance(x, Arm):
				x.set_client(self.arms)
			elif isinstance(x, Grip):
				x.set_client(self.grips)

class ArmController():
	"""Singleton controller class"""
	class __impl():
		def __init__(self, file):
			data = yaml.load(open(file))
			self.config = data[0]
			self.actions = data[1:]
			
			self.config.set_clients(self.actions)
		
		def run():
			for action in self.actions:
				action.send()
	
	__instance = None
	def __init__(self, file):
		if ArmController.__instance == None:
			ArmController.__instance = ArmController.__impl(file)
		self.__dict__['_ArmController__instance'] = ArmController.__instance
	
	def __getattr__(self, attr):
		""" Delegate access to implementation """
		return getattr(self.__instance, attr)
	
	def __setattr__(self, attr, value):
		""" Delegate access to implementation """
		return setattr(self.__instance, attr, value)
	

if __name__ == "__main__":
	controller = ArmController()
	controller.run()