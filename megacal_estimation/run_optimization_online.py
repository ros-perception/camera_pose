#! /usr/bin/env python

import roslib
roslib.load_manifest('megacal_estimation')

import sys, time, optparse
import itertools
import collections

import PyKDL
from tf_conversions import posemath
from calibration_msgs.msg import *

from megacal_estimation.msg import CalibrationEstimate
from megacal_estimation.msg import CameraPose
from megacal_estimation import init_optimization_prior
from megacal_estimation import estimate


class Estimator:
    def __init__(self):
        self.sub = rospy.Subscriber('robot_measurements', self.meas_cb)
        self.state = None
        self.meas = []
        cal_estimate.targets = [ posemath.toMsg(checkerboard_poses[i]) for i in range(len(checkerboard_poses)) ]
        cal_estimate.cameras = [ CameraPose(camera_id, posemath.toMsg(camera_pose)) for camera_id, camera_pose in camera_poses.iteritems()]

    def meas_cb(self, msg):
        # add measurements to list
        self.meas.append(msg)

        # initialize state if needed
        if not self.state:
            self.state = CalibrationEstimate()
            camera_poses, checkerboard_poses = init_optimization_prior.find_initial_poses(self.meas)
            self.state.targets = [ posemath.toMsg(c) for c in checkerboard_poses ]
            self.state.cameras = [ CameraPose(camera_id, posemath.toMsg(camera_pose)) for camera_id, camera_pose in camera_poses.iteritems()]

        # run optimization
        self.state = estimate.enhance(self.meas, self.state)


def main():
    rospy.init_node('online_calibration')
    e = Estimator()
    
    rospy.spin()





