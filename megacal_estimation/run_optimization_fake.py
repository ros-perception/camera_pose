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
#camera_a.pose = posemath.toMsg(PyKDL.Frame(PyKDL.Rotation.RPY(0, pi/2.0, 0), PyKDL.Vector(0, 0 ,0)))
camera_a.pose = posemath.toMsg(PyKDL.Frame(PyKDL.Vector(0, 0 ,0)))

camera_b = CameraPose()
camera_b.camera_id = 'cam_b'
#camera_b.pose = posemath.toMsg(PyKDL.Frame(PyKDL.Rotation.RPY(0,pi/2.0,0), PyKDL.Vector(0, -1, 0)))
camera_b.pose = posemath.toMsg(PyKDL.Frame(PyKDL.Vector(0, -1, 0)))

#target = posemath.toMsg(PyKDL.Frame(PyKDL.Rotation.RPY(0, pi/2.0, 0), PyKDL.Vector(1, 0, 0)))
target = posemath.toMsg(PyKDL.Frame(PyKDL.Vector(0, 0, 1)))

cal_estimate.cameras = [camera_a, camera_b]
cal_estimate.targets = [target]

print cal_estimate


# generate samples
scale = 0.00
offset = PyKDL.Vector(0, 0.001, 0)
cal_sample = RobotMeasurement()
P = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0]
P_mat = reshape( matrix(P, float), (3,4) )

cal_pattern = CalibrationPattern()
for c in check_points:
    pnt = Point32()
    pnt.x = c[0]
    pnt.y = c[1]
    pnt.z = c[2]
    cal_pattern.object_points.append(pnt)

meas_a = CameraMeasurement()
meas_a.camera_id = 'cam_a'
for pnt_c in check_points:
    pnt_a = posemath.fromMsg(camera_a.pose).Inverse() * posemath.fromMsg(target) * pnt_c
    p = P_mat * matrix([pnt_a[0], pnt_a[1], pnt_a[2], 1]).T
    pnt = ImagePoint()
    pnt.x = (p[0]/p[2]) + random.random()*scale
    pnt.y = (p[1]/p[2]) + random.random()*scale
    pnt.d = 1
    meas_a.image_points.append(pnt)
meas_a.cam_info.P = P
meas_a.features = cal_pattern

meas_b = CameraMeasurement()
meas_b.camera_id = 'cam_b'
for pnt_c in check_points:
    pnt_b = posemath.fromMsg(camera_b.pose).Inverse() * posemath.fromMsg(target) * pnt_c
    p = P_mat * matrix([pnt_b[0], pnt_b[1], pnt_b[2], 1]).T
    pnt = ImagePoint()
    pnt.x = (p[0]/p[2]) + random.random()*scale
    pnt.y = (p[1]/p[2]) + random.random()*scale
    pnt.d = 1
    meas_b.image_points.append(pnt)
meas_b.cam_info.P = P
meas_b.features = cal_pattern

cal_sample.M_cam = [meas_a, meas_b]

print cal_sample

# modify prior
target = posemath.toMsg(posemath.fromMsg(target) * PyKDL.Frame(offset))
cal_estimate.targets = [target]

# Run optimization
estimate.enhance([cal_sample], cal_estimate)


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
