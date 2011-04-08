#! /usr/bin/env python

import sys, time, optparse
import itertools
import collections
from numpy import *
import random

import roslib
roslib.load_manifest('megacal_estimation')
import PyKDL
from tf_conversions import posemath

from geometry_msgs.msg import Point32
from calibration_msgs.msg import RobotMeasurement, CameraMeasurement, ImagePoint, CalibrationPattern
from megacal_estimation.msg import CalibrationEstimate, CameraPose

from megacal_estimation import init_optimization_prior
from megacal_estimation import estimate



# checkerboard points in checkerboard frame
check_points = []
for x in range(0,2):
    for y in range(-1, 1):
        check_points.append(PyKDL.Vector(x, y, 1))


# generate camera's and targets
cal_estimate = CalibrationEstimate()

camera_a = CameraPose()
camera_a.camera_id = 'cam_a'
camera_a.pose = posemath.toMsg(PyKDL.Frame(PyKDL.Rotation.RPY(0, pi/2.0, 0), PyKDL.Vector(0, 0 ,0)))
#camera_a.pose = posemath.toMsg(PyKDL.Frame(PyKDL.Vector(0, 0 ,0)))

camera_b = CameraPose()
camera_b.camera_id = 'cam_b'
camera_b.pose = posemath.toMsg(PyKDL.Frame(PyKDL.Rotation.RPY(0,pi/2.0,0), PyKDL.Vector(0, -1, 0)))
#camera_b.pose = posemath.toMsg(PyKDL.Frame(PyKDL.Vector(0, -1, 0)))

target_1 = posemath.toMsg(PyKDL.Frame(PyKDL.Rotation.RPY(0, pi/7.0, 0), PyKDL.Vector(1, 0, 0)))
#target_1 = posemath.toMsg(PyKDL.Frame(PyKDL.Vector(0, 0, 1)))
target_2 = posemath.toMsg(PyKDL.Frame(PyKDL.Rotation.RPY(0, pi/3.0, 0), PyKDL.Vector(2, 0, 0)))
#target_2 = posemath.toMsg(PyKDL.Frame(PyKDL.Vector(0, 0, 1)))

cal_estimate.cameras = [camera_a, camera_b]
cal_estimate.targets = [target_1, target_2]
print cal_estimate



# generate samples
scale = 0.00
offset = PyKDL.Vector(0, 0.1, 0)
P = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0]
P_mat = reshape( matrix(P, float), (3,4) )

cal_pattern = CalibrationPattern()
for c in check_points:
    pnt = Point32()
    pnt.x = c[0]
    pnt.y = c[1]
    pnt.z = c[2]
    cal_pattern.object_points.append(pnt)

cal_samples = []
for target in cal_estimate.targets:
    cal_sample = RobotMeasurement()
    for camera in cal_estimate.cameras:
        meas = CameraMeasurement()    
        meas.camera_id = camera.camera_id
        for pnt_c in check_points:
            pnt_msg = posemath.fromMsg(camera.pose).Inverse() * posemath.fromMsg(target) * pnt_c
            pnt_mat = P_mat * matrix([pnt_msg[0], pnt_msg[1], pnt_msg[2], 1]).T
            pnt = ImagePoint()
            pnt.x = (pnt_mat[0]/pnt_mat[2]) + random.random()*scale
            pnt.y = (pnt_mat[1]/pnt_mat[2]) + random.random()*scale
            pnt.d = 1
            meas.image_points.append(pnt)
        meas.cam_info.P = P
        meas.features = cal_pattern
        cal_sample.M_cam.append(meas)
    cal_samples.append(cal_sample)
print cal_samples


# add offset
cal_estimate.cameras[0].pose = posemath.toMsg(posemath.fromMsg(cal_estimate.cameras[0].pose) * PyKDL.Frame(offset))


# Run optimization
estimate.enhance(cal_samples, cal_estimate)


# Just run oplus & sub h

# print "---------- Vijay's Minimal Test ----------"
# set_printoptions(linewidth=300, precision=8, suppress=True)
# from sensor_msgs.msg import CameraInfo
# cam_pose = PyKDL.Frame(PyKDL.Vector(0,0,0))
# target_pose = PyKDL.Frame(PyKDL.Vector(0,0,1))
# target_pts = [ [ 0, 0, 1 ],
#                [-1, 0,-1 ],
#                [ 1, 1, 1 ],
#                [ 1, 1, 1] ]
# cam_info = CameraInfo()
# cam_info.P = [1, 0, 0, 0,
#               0, 1, 0, 0,
#               0, 0, 1, 0]
# before = estimate.sub_h(cam_pose, target_pose, target_pts, cam_info)
# eps = 1e-6
# step = matrix([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).transpose()
# step[3,0] = eps
# print "Step:\n", step
# cam_pose_1 = estimate.pose_oplus(cam_pose, step)
# after = estimate.sub_h(cam_pose_1, target_pose, target_pts, cam_info)

# print "Cam Pose Before:\n", cam_pose
# print "Cam Pose After:\n", cam_pose_1

# print "Before:\n", before
# print "After:\n", after

# J = (after - before) / eps
# print "J:\n", J
