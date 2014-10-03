#! /usr/bin/env python

import sys, time, optparse
import itertools
import collections

import PyKDL
from tf_conversions import posemath
from calibration_msgs.msg import *

from camera_pose_calibration.msg import CalibrationEstimate
from camera_pose_calibration.msg import CameraPose

import rosbag
from camera_pose_calibration import init_optimization_prior
from camera_pose_calibration import estimate
from camera_pose_calibration import dump_estimate

import yaml
import sys

# read data from bag
if len(sys.argv) >= 2:
    BAG = sys.argv[1]
else:
    BAG = '/u/vpradeep/kinect_bags/kinect_extrinsics_2011-04-05-16-01-28.bag'
bag = rosbag.Bag(BAG)
for topic, msg, t in bag:
    assert topic == 'robot_measurement'
cal_samples = [msg for topic, msg, t in bag]


# create prior
camera_poses, checkerboard_poses = init_optimization_prior.find_initial_poses(cal_samples)
cal_estimate = CalibrationEstimate()
cal_estimate.targets = [ posemath.toMsg(checkerboard_poses[i]) for i in range(len(checkerboard_poses)) ]
cal_estimate.cameras = [ CameraPose(camera_id, posemath.toMsg(camera_pose)) for camera_id, camera_pose in camera_poses.iteritems()]


# Run optimization
new_cal_estimate = estimate.enhance(cal_samples, cal_estimate)
cam_dict_list = dump_estimate.to_dict_list(new_cal_estimate.cameras)

# For now, hardcode what transforms we care about
tf_config = dict();
tf_config['camera_a'] = {'calibrated_frame':'camera_a/openni_rgb_optical_frame',
                         'parent_frame': 'world_frame',
                         'child_frame': 'camera_a/openni_camera'}
tf_config['camera_b'] = {'calibrated_frame':'camera_b/openni_rgb_optical_frame',
                         'parent_frame': 'world_frame',
                         'child_frame': 'camera_b/openni_camera'}

# Insert TF Data into output file
for cam_dict in cam_dict_list:
    cam_id = cam_dict['camera_id']
    if cam_id in tf_config:
        cam_dict['tf'] = tf_config[cam_id]
    else:
        cam_dict['tf'] = {'calibrated_frame': cam_id,
                          'parent_frame': 'world_frame',
                          'child_frame': cam_id}

f = open('camera_cal.yaml', 'w')
f.write(yaml.dump(cam_dict_list))
f.close()
