#! /usr/bin/env python

import sys, time, optparse
import itertools
import collections
from numpy import *

import roslib
roslib.load_manifest('megacal_estimation')
import PyKDL
from tf_conversions import posemath

from geometry_msgs.msg import Point32
from calibration_msgs.msg import RobotMeasurement, CameraMeasurement, ImagePoint, CalibrationPattern
from megacal_estimation.msg import CalibrationEstimate, CameraPose

from megacal_estimation import init_optimization_prior
from megacal_estimation import estimate



# generate camera's and targets
cal_estimate = CalibrationEstimate()

camera_a = CameraPose()
camera_a.camera_id = 'cam_a'
camera_a.pose = posemath.toMsg(PyKDL.Frame())

camera_b = CameraPose()
camera_b.camera_id = 'cam_b'
camera_b.pose = posemath.toMsg(PyKDL.Frame(PyKDL.Vector(0, -1, 0)))

target = posemath.toMsg(PyKDL.Frame(PyKDL.Vector(0, 0, 1)))

cal_estimate.cameras = [camera_a, camera_b]
cal_estimate.targets = [target]

print cal_estimate


# generate samples
cal_sample = RobotMeasurement()
P = [1, 0, 1, 0, 0, 1, 1, 0, 0, 0, 0, 1]
P_mat = reshape( matrix(P, float), (3,4) )

cal_pattern = CalibrationPattern()
for x in range(0,2):
    for y in range(-1, 1):
        pnt = Point32()
        pnt.x = x
        pnt.y = y
        pnt.z = 1
        cal_pattern.object_points.append(pnt)


meas_a = CameraMeasurement()
meas_a.camera_id = 'cam_a'
for x in range(0,2):
    for y in range(-1, 1):
        p = P_mat * matrix([x, y, 2, 1]).T
        pnt = ImagePoint()
        pnt.x = p[0]
        pnt.y = p[1]        
        pnt.d = 1
        meas_a.image_points.append(pnt)
meas_a.cam_info.P = P
meas_a.features = cal_pattern

meas_b = CameraMeasurement()
meas_b.camera_id = 'cam_b'
for x in range(0,2):
    for y in range(0, 2):
        p = P_mat * matrix([x, y, 2, 1]).T
        pnt = ImagePoint()
        pnt.x = p[0]
        pnt.y = p[1]        
        pnt.d = 1
        meas_b.image_points.append(pnt)
meas_b.cam_info.P = P
meas_b.features = cal_pattern

cal_sample.M_cam = [meas_a, meas_b]

print cal_sample

# Run optimization
estimate.enhance([cal_sample], cal_estimate)
