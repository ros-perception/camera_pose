#! /usr/bin/env python

import sys, time, optparse
import itertools
import collections
from numpy import *
import random

import PyKDL
from tf_conversions import posemath

from geometry_msgs.msg import Point32
from calibration_msgs.msg import ImagePoint, CalibrationPattern
from camera_pose_calibration.msg import CalibrationEstimate, CameraPose, RobotMeasurement, CameraMeasurement

from camera_pose_calibration import init_optimization_prior
from camera_pose_calibration import estimate



# checkerboard points in checkerboard frame
check_points = []
for x in range(0, 2):
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

#target_1 = posemath.toMsg(PyKDL.Frame(PyKDL.Rotation.RPY(0, pi/7.0, 0), PyKDL.Vector(1, 0, 0)))
target_1 = posemath.toMsg(PyKDL.Frame(PyKDL.Vector(0, -2, 1)))
#target_2 = posemath.toMsg(PyKDL.Frame(PyKDL.Rotation.RPY(0, pi/3.0, 0), PyKDL.Vector(2, 0, 0)))
target_2 = posemath.toMsg(PyKDL.Frame(PyKDL.Vector(0, 2, 1)))

cal_estimate.cameras = [camera_a, camera_b]
cal_estimate.targets = [target_1, target_2]
print cal_estimate


# generate samples
noise = 5.0
offset = PyKDL.Frame(PyKDL.Rotation.RPY(0.1, 0.1, 0), PyKDL.Vector(0, 0.1, 0))

#P = [1,    0,   0,      0,     0,   1,     0,      0,     0, 0, 1, 0]
P = [525,   0,   319.5,  0,     0,   525,   239.5,  0,     0, 0, 1,  0]
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
            pnt.x = (pnt_mat[0]/pnt_mat[2]) + random.random()*noise
            pnt.y = (pnt_mat[1]/pnt_mat[2]) + random.random()*noise
            pnt.d = 1
            meas.image_points.append(pnt)
        meas.cam_info.P = P
        meas.features = cal_pattern
        cal_sample.M_cam.append(meas)
    cal_samples.append(cal_sample)
print cal_samples


# add offset
cal_estimate.cameras[0].pose = posemath.toMsg(posemath.fromMsg(cal_estimate.cameras[0].pose) * offset)
#cal_estimate.cameras[1].pose = posemath.toMsg(posemath.fromMsg(cal_estimate.cameras[1].pose) * PyKDL.Frame(offset))

# Run optimization
estimate.enhance(cal_samples, cal_estimate)

#residual, J = estimate.calculate_residual_and_jacobian(cal_samples, cal_estimate)

#set_printoptions(linewidth=300, precision=5, suppress=True)
#print "Jacobian:\n", J
#print "Residual:\n", reshape(residual, [-1,2])

#step = linalg.pinv(J)*residual
#print "Unscaled Step:\n", reshape(step, [-1, 6]).T
#print "RMS: ", estimate.rms(residual)
#next_estimate = estimate.oplus(cal_estimate, step)
#print "Next Estimate:\n", next_estimate
