# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# author: Vijay Pradeep

# Define a camera sensor attached to a chain
#
#       before_chain_Ts -- camera_chain -- after_chain_Ts -- camera
#      /
#   root
#      \
#       checkerboard


import numpy
from numpy import matrix, reshape, array, zeros, diag, real

import roslib; roslib.load_manifest('pr2_calibration_estimation')
import rospy
from pr2_calibration_estimation.full_chain import FullChainRobotParams
from sensor_msgs.msg import JointState

class CameraChainBundler:
    def __init__(self, valid_configs):
        self._valid_configs = valid_configs

    # Construct a CameraChainSensor for every camera chain sensor that exists in the given robot measurement
    def build_blocks(self, M_robot):
        sensors = []
        for cur_config in self._valid_configs:
            if cur_config["camera_id"] in [ x.camera_id for x in M_robot.M_cam ] and \
               cur_config["chain"]["chain_id"] in [ x.chain_id  for x in M_robot.M_chain ] :
                M_cam   = M_robot.M_cam  [ [ x.camera_id for x in M_robot.M_cam   ].index(cur_config["camera_id"])]
                M_chain = M_robot.M_chain[ [ x.chain_id  for x in M_robot.M_chain ].index(cur_config["chain"]["chain_id"]) ]
                cur_sensor = CameraChainSensor(cur_config, M_cam, M_chain)
                sensors.append(cur_sensor)
            else:
                rospy.logdebug("  Didn't find block")
        return sensors

class CameraChainSensor:
    def __init__(self, config_dict, M_cam, M_chain):

        self.sensor_type = "camera"
        self.sensor_id = config_dict["camera_id"]

        self._config_dict = config_dict
        self._M_cam = M_cam
        self._M_chain = M_chain

        self._chain = FullChainRobotParams(config_dict["chain"])

        self.terms_per_sample = 2

    def update_config(self, robot_params):
        self._camera = robot_params.rectified_cams[ self._config_dict["camera_id"] ]

        self._chain.update_config(robot_params)

    def compute_residual(self, target_pts):
        z_mat = self.get_measurement()
        h_mat = self.compute_expected(target_pts)
        assert(z_mat.shape[1] == 2)
        assert(h_mat.shape[1] == 2)
        assert(z_mat.shape[0] == z_mat.shape[0])
        r = array(reshape(h_mat - z_mat, [-1,1]))[:,0]
        return r


    def compute_residual_scaled(self, target_pts):
        r = self.compute_residual(target_pts)
        gamma_sqrt = self.compute_marginal_gamma_sqrt(target_pts)
        r_scaled = gamma_sqrt * matrix(r).T
        return array(r_scaled.T)[0]

    def compute_marginal_gamma_sqrt(self, target_pts):
        import scipy.linalg
        cov = self.compute_cov(target_pts)
        gamma = matrix(zeros(cov.shape))
        num_pts = self.get_residual_length()/2

        for k in range(num_pts):
            #print "k=%u" % k
            first = 2*k
            last = 2*k+2
            sub_cov = matrix(cov[first:last, first:last])
            sub_gamma_sqrt_full = matrix(scipy.linalg.sqrtm(sub_cov.I))
            sub_gamma_sqrt = real(sub_gamma_sqrt_full)
            assert(scipy.linalg.norm(sub_gamma_sqrt_full - sub_gamma_sqrt) < 1e-6)
            gamma[first:last, first:last] = sub_gamma_sqrt
        return gamma

    def get_residual_length(self):
        N = len(self._M_cam.image_points)
        return N*2

    # Get the observed measurement in a Nx2 Matrix
    def get_measurement(self):
        camera_pix = numpy.matrix([[pt.x, pt.y] for pt in self._M_cam.image_points])
        return camera_pix

    # Compute the expected pixel coordinates for a set of target points.
    # target_pts: 4xN matrix, storing feature points of the target, in homogeneous coords
    # Returns: target points projected into pixel coordinates, in a Nx2 matrix
    def compute_expected(self, target_pts):
        return self._compute_expected(self._M_chain.chain_state, target_pts)

    def _compute_expected(self, chain_state, target_pts):
        # Camera pose in root frame
        camera_pose_root = self._chain.calc_block.fk(chain_state)
        cam_frame_pts = camera_pose_root.I * target_pts
        # Do the camera projection
        pixel_pts = self._camera.project(self._M_cam.cam_info.P, cam_frame_pts)

        return pixel_pts.T

    def compute_expected_J(self, target_pts):
        epsilon = 1e-8
        N = len(self._M_cam.image_points)
        Jt = zeros([N*3, N*2])
        for k in range(N):
            # Compute jacobian for point k
            sub_Jt = zeros([3,2])
            x = target_pts[:,k].copy()
            f0 = self.compute_expected(x)
            for i in [0,1,2]:
                x[i,0] += epsilon
                fTest = self.compute_expected(x)
                sub_Jt[i,:] = array((fTest - f0) / epsilon)[:,0]
                x[i,0] -= epsilon
            Jt[k*3:(k+1)*3, k*2:(k+1)*2] = sub_Jt
        return Jt.T


    def compute_cov(self, target_pts):
        epsilon = 1e-8

        num_joints = len(self._M_chain.chain_state.position)
        Jt = zeros([num_joints, self.get_residual_length()])

        x = JointState()
        x.position = self._M_chain.chain_state.position[:]

        f0 = reshape(array(self._compute_expected(x, target_pts)), [-1])
        for i in range(num_joints):
            x.position = [cur_pos for cur_pos in self._M_chain.chain_state.position]
            x.position[i] += epsilon
            fTest = reshape(array(self._compute_expected(x, target_pts)), [-1])
            Jt[i] = (fTest - f0)/epsilon
        cov_angles = [x*x for x in self._chain.calc_block._chain._cov_dict['joint_angles']]
        chain_cov = matrix(Jt).T * matrix(diag(cov_angles)) * matrix(Jt)
        cam_cov = matrix(zeros(chain_cov.shape))
        # Convert StdDev into variance
        var_u = self._camera._cov_dict['u'] * self._camera._cov_dict['u']
        var_v = self._camera._cov_dict['v'] * self._camera._cov_dict['v']
        for k in range(cam_cov.shape[0]/2):
            cam_cov[2*k  , 2*k]   = var_u
            cam_cov[2*k+1, 2*k+1] = var_v

        #import code; code.interact(local=locals())
        cov = chain_cov + cam_cov
        return cov

    # Build a dictionary that defines which parameters will in fact affect this measurement
    def build_sparsity_dict(self):
        sparsity = dict()
        sparsity['transforms'] = {}
        for cur_transform_name in ( self._config_dict['chain']['before_chain'] + self._config_dict['chain']['after_chain'] ):
            sparsity['transforms'][cur_transform_name] = [1, 1, 1, 1, 1, 1]

        sparsity['dh_chains'] = {}
        chain_id = self._config_dict['chain']['chain_id']
        num_links = self._chain.calc_block._chain._M
        assert(num_links == len(self._M_chain.chain_state.position))
        sparsity['dh_chains'][chain_id] = [ [1,1,1,1] ] * num_links

        sparsity['rectified_cams'] = {}
        sparsity['rectified_cams'][self.sensor_id] = {'baseline_shift': 1}

        return sparsity


