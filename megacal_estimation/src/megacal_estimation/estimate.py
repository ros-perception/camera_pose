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
import megacal_estimation.msg.CalibrationEstimate
import megacal_estimation.msg.CameraPose
from numpy import *

pose_width = 6
feature_width = 2

def estimate(cal_samples, prior_estimate):










    return best_estimate


def calculate_jacobian(cal_samples, cur_estimate):

    # Compute the total number of rows. This is the number of poses * 6
    num_cols = len(cur_estimate.cameras) * pose_width + len(cur_estimate.targets) * pose_width
    num_rows = sum ([ sum([  len(cam.image_points) for cam in cur_sample.M_cam]) for cur_sample in cal_samples]) * feature_width

    J = zeros([num_rows, num_cols])

    cam_pose_dict  = dict( [ (cur_camera.camera_id, cur_camera.pose)  for cur_camera in cur_estimate.cameras] )
    cam_index_dict = dict( [ (cur_camera.camera_id, cur_index)        for cur_camera, cur_index  in zip ( cur_estimate.cameras, range(cur_estimate.cameras) )] )

    target_col_offset = len(cur_estimate.cameras) * pose_width

    # Start filling in the jacobian
    cur_row = 0;
    # Loop over each observation
    for cur_sample, target_pose, target_index in zip(cal_samples, cur_estimate.targets, range(len(cur_estimate.targets))):
        for cam_measurement in cur_sample.M_cam:
            # Find the index of this camera
            try:
                cam_pose  = cam_pose_dict[cam_measurement.camera_id]
                cam_index = cam_index_dict[cam_measurement.camera_id]
            except KeyError:
                print "Couldn't find current camera_id in cam_pose dictionary"
                print "  camera_id = %s", cam_measurement.camera_id
                print "  %s", cam_pose_dict.keys()
                raise

            camera_J = calculate_camera_jacobian(cam_measurement, cam_pose, target_pose)
            target_J = calculate_target_jacobian(can_measurement, cam_pose, target_pose)

            end_row = cur_row + len(cam_measurement.image_points)*feature_width;

            # Insert camera jacobian into big matrix
            J[cur_row:end_row, cam_index*pose_width:((cam_index+1)*pose_width)] = camera_J;

            # Insert target jacobian into big matrix
            J[cur_row:end_row, targets_col_offset + target_index*pose_width:(target_index+1)*pose_width] = target_J;

            cur_row = end_row
    return J

def calculate_sub_jacobian(cam_measurement, cam_pose_msg, target_pose_msg, use_cam):
    """
    use_cam: True  -> compute J for the camera pose
             False -> compute J for the target pose
    """
    # ROS Poses  -> KDL Poses
    cam_pose    = posemath.fromMsg(cam_pose_msg)
    target_pose = posemath.fromMsg(target_pose_msg)

    target_pose_1 = target_pose
    cam_pose_1    = cam_pose

    # ROS Target Points -> (4xN) Homogenous coords
    target_pts = matrix([ [pt.x, pt.y, pt.z, 1.0] for pt in cam_measurement.features.object_points ]).transpose()

    # ROS Pixel Measurements -> 2Nx1 Matrix
    pixel_measurement = matrix( concatentate ( [ [pt.x, pt.y] for pt in cam_measurement.image_points ] )).transpose()

    m0 = h(cam_pose, target_pose, target_pts, cam_measurement.cam_info)

    J = zeros([ len(cam_measurement.image_points)*feature_width, pose_width])

    eps = 1e-5
    for axis in range(pose_width):
        t = PyKDL.Twist()
        t[axis] = eps
        # Decide if we're operating on camera poses or target poses
        if use_cam:
            cam_pose_1 = PyKDL.addDelta(cam_pose, t, 1.0)
        else:
            target_pose_1 = PyKDL.addDelta(target_pose, t, 1.0)
        m1 = h(cam_pose_1, target_pose_1, target_pts, cam_measurement.cam_info)
        J[:, axis] = (m1 - m0) / eps

    return J

def h(cam_pose, target_pose, target_pts, cam_info):
    '''
    P:   3x4 Camera Projection Matrix
    target_pts: 4xN matrix, storing feature points of the target, in homogeneous coords
    return: 2Nx1 matrix of pixel coordinates
    '''
    # CameraInfo -> Numpy Camera Projection Matrix
    P = reshape( matrix(cam_info.P, float), (3,4) )

    P_full = P * to4x4(cam_pose.Inverse() * target_pose)

    # Apply projection matrix
    pixel_pts_h = P_full * pts

    # Strip out last row (3rd) and rescale
    #pixel_pts = pixel_pts_h[0:2,:] / pixel_pts_h[2,:]
    pixel_pts_flat = concatenate( [pixel_pts_h[0, :]/pixel_pts_h[2, :],
                                   pixel_pts_h[1, :]/pixel_pts_h[2, :]], 1 )

    return pixel_pts_flat.T

def to4x4(kdl_frame):
    # Init output matrix
    T = matrix( zeros((4,4), float ))
    T[3,3] = 1.0

    # Copy position into matrix
    T[0:3,3] = [ kdl_frame.p.[0], kdl_frame.p.[1], kdl_frame.p.[2] ];

    # Generate pointer to rotation submatrix
    R = T[0:3,0:3]
    for i in range(3):
        for j in range(3):
            R[i,j] = kdl_frame.M[i,j]
    return T

def oplus(kdl_frame, update):
    '''
    update: parameter vector of size 6
    dims:
      0 -> Translate x
      1 -> Translate y
      2 -> Translate z
      3 -> Rot x
      4 -> Rot y
      5 -> Rot z
    '''
    t = PyKDL.Twist( PyKDL.Vector(*update[:3]), PyKDL.Vector(*update[3:]) )
    return PyKDL.addDelta(kdl_frame, t, 1.0)
