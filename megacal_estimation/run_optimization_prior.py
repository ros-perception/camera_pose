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

BAG = '/u/vpradeep/kinect_bags/kinect_extrinsics_2011-04-05-16-01-28.bag'
camera_poses, checkerboard_poses = init_optimization_prior.find_initial_poses(BAG)

cal_estimate = CalibrationEstimate()
cal_estimate.targets = [ posemath.toMsg(checkerboard_poses[i]) for i in range(len(checkerboard_poses)) ]
cal_estimate.targets = [ cal_estimate.targets[0] ]

cal_estimate.cameras = [ CameraPose(camera_id, posemath.toMsg(camera_pose)) for camera_id, camera_pose in camera_poses.iteritems()]
print cal_estimate


# Run optimization
bag = rosbag.Bag(BAG)
for topic, msg, t in bag:
    assert topic == 'robot_measurement'
cal_samples = [msg for topic, msg, t in bag]
cal_samples = [cal_samples[0]]


estimate.enhance(cal_samples, cal_estimate)
