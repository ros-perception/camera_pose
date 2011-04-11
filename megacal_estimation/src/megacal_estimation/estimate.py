# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import roslib; roslib.load_manifest('megacal_estimation')

import PyKDL
from tf_conversions import posemath
from megacal_estimation.msg import CalibrationEstimate
from megacal_estimation.msg import CameraPose
from numpy import *
from copy import deepcopy

pose_width = 6
feature_width = 2
num_iterations = 200000
step_scale = 0.0000001


def enhance(cal_samples, prior_estimate):
    set_printoptions(linewidth=300, precision=5, suppress=True)
    next_estimate = deepcopy(prior_estimate)
    for i in range(num_iterations):
        residual, J = calculate_residual_and_jacobian(cal_samples, next_estimate)
        #step = linalg.pinv(J)*residual
        step = J.T*residual
        next_estimate = oplus(next_estimate, step*step_scale)

        print "RMS: ", rms(residual)
#        print "Residual: \n",residual.T
#        print "Jacobian: \n",J
#        print "Correction: \n",step.T
        


def rms(residual):
    return sqrt(rms_sq(residual))

def rms_sq(residual):
    return sum(array(residual) * array(residual) / len(residual))


def pose_oplus(kdl_pose, step):
    t = PyKDL.Twist()
    for i in range(pose_width):
        t[i] = step[i,0]
    next_pose = kdl_pose * PyKDL.addDelta(PyKDL.Frame(), t, 1.0)
    return next_pose


def oplus(cur_estimate, step):
    result = deepcopy(cur_estimate)

    # loop over camera's
    for camera, res, camera_index in zip(cur_estimate.cameras, result.cameras, [r*pose_width for r in range(len(cur_estimate.cameras))]):
        res.pose = posemath.toMsg(pose_oplus(posemath.fromMsg(camera.pose), step[camera_index:camera_index+pose_width]))

    # loop over targets
    for target, res_index, target_index in zip(cur_estimate.targets, range(len(result.targets)), [(r+len(cur_estimate.cameras))*pose_width for r in range(len(cur_estimate.targets))]):
        result.targets[res_index] = posemath.toMsg(pose_oplus(posemath.fromMsg(target), step[target_index:target_index+pose_width]))

    return result

def calculate_residual_and_jacobian(cal_samples, cur_estimate):
    """
    returns the full residual vector and jacobian
    """
    # Compute the total number of rows. This is the number of poses * 6
    num_cols = len(cur_estimate.cameras) * pose_width + len(cur_estimate.targets) * pose_width
    num_rows = sum ([ sum([  len(cam.image_points) for cam in cur_sample.M_cam]) for cur_sample in cal_samples]) * feature_width

    J = matrix(zeros([num_rows, num_cols]))
    residual = matrix(zeros([num_rows, 1]))

    cam_pose_dict  = dict( [ (cur_camera.camera_id, posemath.fromMsg(cur_camera.pose))  for cur_camera in cur_estimate.cameras] )
    cam_index_dict = dict( [ (cur_camera.camera_id, cur_index)        for cur_camera, cur_index  in zip ( cur_estimate.cameras, range(len(cur_estimate.cameras)) )] )

    # Start filling in the jacobian
    cur_row = 0;
    # Loop over each observation
    for cur_sample, target_pose_msg, target_index in zip(cal_samples, cur_estimate.targets, range(len(cur_estimate.targets))):
        for cam_measurement in cur_sample.M_cam:
            # Find the index of this camera
            try:
                cam_pose = cam_pose_dict[cam_measurement.camera_id]
                cam_index     = cam_index_dict[cam_measurement.camera_id]
            except KeyError:
                print "Couldn't find current camera_id in cam_pose dictionary"
                print "  camera_id = %s", cam_measurement.camera_id
                print "  %s", cam_pose_dict.keys()
                raise

            # ROS Poses  -> KDL Poses
            target_pose = posemath.fromMsg(target_pose_msg)

            end_row = cur_row + len(cam_measurement.image_points)*feature_width

            # ROS Target Points -> (4xN) Homogenous coords
            target_pts = matrix([ [pt.x, pt.y, pt.z, 1.0] for pt in cam_measurement.features.object_points ]).transpose()

            # Save the residual for this cam measurement
            measurement_vec = matrix( concatenate([ [cur_pt.x, cur_pt.y] for cur_pt in cam_measurement.image_points]) ).T
            expected_measurement = sub_h(cam_pose, target_pose, target_pts, cam_measurement.cam_info)
            residual[cur_row:end_row, 0] =  measurement_vec - expected_measurement 

            # Compute jacobian for this cam measurement
            camera_J = calculate_sub_jacobian(cam_pose, target_pose, target_pts, cam_measurement.cam_info, use_cam = True)
            #print "Camera Jacobian [%s]]" % cam_measurement.camera_id
            #print camera_J
            target_J = calculate_sub_jacobian(cam_pose, target_pose, target_pts, cam_measurement.cam_info, use_cam = False)
            J[cur_row:end_row, cam_index*pose_width:((cam_index+1)*pose_width)] = camera_J
            col_start = (len(cur_estimate.cameras) + target_index)*pose_width
            J[cur_row:end_row, col_start:(col_start+pose_width)] = target_J

            cur_row = end_row
    return residual, J


def calculate_sub_jacobian(cam_pose, target_pose, target_pts, cam_info, use_cam):
    """
    use_cam: True  -> compute J for the camera pose
             False -> compute J for the target pose
    """

    target_pose_1 = kdlcopy(target_pose)
    cam_pose_1    = kdlcopy(cam_pose)

    m0 = sub_h(cam_pose, target_pose, target_pts, cam_info)
    J = matrix(zeros([ target_pts.shape[1]*feature_width, pose_width]))

    eps = 1e-4
    for axis in range(pose_width):
        step = matrix(zeros([pose_width,1]))
        step[axis,0] = eps;
        # Decide if we're operating on camera poses or target poses
        if use_cam:
            cam_pose_1 =  pose_oplus(cam_pose, step)
            #print "CAM POSE 1\n", cam_pose_1
        else:
            target_pose_1 = pose_oplus(target_pose, step)
            #print "TARGET POSE 1\n", target_pose_1
        m1 = sub_h(cam_pose_1, target_pose_1, target_pts, cam_info)
        #print "SUB_ m1 \n", m1*100000
        #print "SUB_ m0 \n", m0*100000
        #print "Diff \n", (m1 - m0)*100000
        #print "SUB_JAC \n", (m1- m0)/eps
        J[:, axis] = (m1 - m0) / eps

    return J

def kdlcopy(pose):
    f = PyKDL.Frame()
    for i in range(3):
        f.p[i] = pose.p[i]
    for i in range(3):
        for j in range(3):
            f.M[i,j] = pose.M[i,j]
    return f


def sub_h(cam_pose, target_pose, target_pts, cam_info):
    '''
    P:   3x4 Camera Projection Matrix
    target_pts: 4xN matrix, storing feature points of the target, in homogeneous coords
    return: 2Nx1 matrix of pixel coordinates
    '''
    # CameraInfo -> Numpy Camera Projection Matrix
    P = reshape( matrix(cam_info.P, float), (3,4) )
    
    P_full = P * to4x4(cam_pose.Inverse() * target_pose)

    # Apply projection matrix
    pixel_pts_h = P_full * target_pts

    # Strip out last row (3rd) and rescale
    #pixel_pts = pixel_pts_h[0:2,:] / pixel_pts_h[2,:]
    pixel_pts_flat = reshape( concatenate( [ pixel_pts_h[0, :]/pixel_pts_h[2, :],
                                             pixel_pts_h[1, :]/pixel_pts_h[2, :]], 0 ).T, [-1,1] )

    return pixel_pts_flat

def to4x4(kdl_frame):
    # Init output matrix
    T = matrix( zeros((4,4), float ))
    T[3,3] = 1.0

    # Copy position into matrix
    T[0:3,3] = matrix([ kdl_frame.p[0], kdl_frame.p[1], kdl_frame.p[2] ]).T

    # Generate pointer to rotation submatrix
    R = T[0:3,0:3]
    for i in range(3):
        for j in range(3):
            R[i,j] = kdl_frame.M[i,j]
    #print "to4x4 kdl: \n", kdl_frame
    #print "to4x4 mat: \n",T
    return T
