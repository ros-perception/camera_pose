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
from megacal_estimation import generate_urdf


BAG = '/u/vpradeep/kinect_bags/kinect_extrinsics_2011-04-05-16-01-28.bag'
camera_poses, checkerboard_poses = init_optimization_prior.find_initial_poses(BAG)

cal_estimate = CalibrationEstimate()
cal_estimate.targets = [ posemath.toMsg(checkerboard_poses[i]) for i in range(len(checkerboard_poses)) ]
cal_estimate.cameras = [ CameraPose(camera_id, posemath.toMsg(camera_pose)) for camera_id, camera_pose in camera_poses.iteritems()]

print cal_estimate

#print camera_poses
#print "\n\n"
#print checkerboard_poses

# Run optimization

bag = rosbag.Bag(BAG)


for topic, msg, t in bag:
    assert topic == 'robot_measurement'

cal_samples = [msg for topic, msg, t in bag]
#cal_samples = cal_samples[:1]
#cal_samples[0].M_cam = cal_samples[0].M_cam[:1]

residual, J = estimate.calculate_residual_and_jacobian(cal_samples, cal_estimate)

#m = matrix( [[pt.x, pt.y] for pt in cal_samples[0].M_cam[0].image_points] )

print "Residual:\n%s" % residual
print "J:\n%s" % J

#print "RMS Error: %s" % sqrt(sum(array(residual) * array(residual))/(residual.shape[0]*2))

urdf = generate_urdf.generate_urdf(cal_estimate.cameras)

out_filename = 'multinect_urdf.xml'
f = open(out_filename, 'w')
f.write(urdf)
f.close()
print urdf

print "Wrote urdf to [%s]" % out_filename

