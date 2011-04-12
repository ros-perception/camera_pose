#! /usr/bin/env python

import sys, time, optparse
import itertools
import collections

import roslib
roslib.load_manifest('megacal_estimation')
import PyKDL
from tf_conversions import posemath
from calibration_msgs.msg import *

from megacal_estimation.msg import CalibrationEstimate
from megacal_estimation.msg import CameraPose

import rosbag
from megacal_estimation import init_optimization_prior
from megacal_estimation import estimate
import sys
BAG = sys.argv[1]
camera_poses, checkerboard_poses = init_optimization_prior.find_initial_poses(BAG)

cal_estimate = CalibrationEstimate()
cal_estimate.targets = [ posemath.toMsg(checkerboard_poses[i]) for i in range(len(checkerboard_poses)) ]
cal_estimate.cameras = [ CameraPose(camera_id, posemath.toMsg(camera_pose)) for camera_id, camera_pose in camera_poses.iteritems()]

# Run optimization
bag = rosbag.Bag(BAG)
for topic, msg, t in bag:
    assert topic == 'robot_measurement'
cal_samples = [msg for topic, msg, t in bag]


estimate.enhance(cal_samples, cal_estimate)
