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

# Error block for a monocular camera plus tilting laser.  The
# camera is attached to a chain, but also has a 6 dof transform
# in between the tilting laser is attached to the
#
#       before_chain_Ts -- camera_chain -- after_chain_Ts -- camera
#      /
#   root
#      \
#       before_chain_Ts -- target_chain -- after_chain_Ts -- checkerboard

import numpy
from numpy import matrix
import rospy
from pr2_calibration_estimation.full_chain import FullChainRobotParams

class CameraChainBundler:
    def __init__(self, valid_configs):
        self._valid_configs = valid_configs

    # Construct a CameraChainRobotParamsBlock for every 'valid config' that finds everything it needs in the current robot measurement
    def build_blocks(self, M_robot):
        blocks = []
        #import code; code.interact(local=locals())
        for cur_config in self._valid_configs:
            #print         ("On Config Block [%s]-[%s]-[%s]" % (cur_config["camera_id"], \
            #                                                   cur_config["camera_chain"]["chain_id"], \
            #                                                   cur_config["target_chain"]["chain_id"]) )

            #print "  Target Chain: %s" % M_robot.chain_id
            #print "  Camera IDs: %s" % ", ".join([x.camera_id for x in M_robot.M_cam])
            #print "  Chains: %s" % ", ".join([ x.chain_id  for x in M_robot.M_chain ])

            if cur_config["target_chain"]["chain_id"] == M_robot.chain_id and \
               cur_config["camera_id"] in [ x.camera_id for x in M_robot.M_cam ] and \
               cur_config["camera_chain"]["chain_id"] in [ x.chain_id  for x in M_robot.M_chain ] and \
               cur_config["target_chain"]["chain_id"] in [ x.chain_id  for x in M_robot.M_chain ] :
                rospy.logdebug("  Found block")
                M_cam       = M_robot.M_cam  [ [ x.camera_id for x in M_robot.M_cam   ].index(cur_config["camera_id"])]
                M_camera_chain = M_robot.M_chain[ [ x.chain_id  for x in M_robot.M_chain ].index(cur_config["camera_chain"]["chain_id"]) ]
                M_target_chain = M_robot.M_chain[ [ x.chain_id  for x in M_robot.M_chain ].index(cur_config["target_chain"]["chain_id"]) ]
                cur_block = CameraChainRobotParamsBlock(cur_config,
                                                        M_cam, M_camera_chain, M_target_chain,
                                                        M_robot.target_id)
                cur_block.error_scalar = cur_config["error_scalar"]
                blocks.append(cur_block)
            else:
                rospy.logdebug("  Didn't find block")
        return blocks

class CameraChainRobotParamsBlock:
    def __init__(self, config_dict, M_cam, M_camera_chain, M_target_chain, target_id):

        self.block_type = "CameraChain"
        self.block_id = config_dict["block_id"]

        self._config_dict = config_dict
        self._M_cam = M_cam
        self._M_camera_chain = M_camera_chain
        self._M_target_chain = M_target_chain
        self._target_id = target_id

        self._full_camera_chain = FullChainRobotParams(config_dict["camera_chain"])
        self._full_target_chain = FullChainRobotParams(config_dict["target_chain"])
        self._calc_block = CameraChainCalcBlock()

    def update_config(self, robot_params):
        camera = robot_params.rectified_cams[ self._config_dict["camera_id"] ]
        cb     = robot_params.checkerboards[ self._target_id ]

        self._full_camera_chain.update_config(robot_params)
        self._full_target_chain.update_config(robot_params)
        self._calc_block.update_config(self._full_camera_chain.calc_block,
                                       self._full_target_chain.calc_block,
                                       camera, cb)

    def compute_error(self):
        error_mat = self._calc_block.compute_error(self._M_target_chain, self._M_camera_chain, self._M_cam)
        #error_vec = numpy.reshape(error_mat, (-1, 1))
        #return error_vec
        return error_mat

    # Plotting helpers
    def compute_expected(self):
        return self._calc_block.compute_expected(self._M_cam)

    def compute_actual(self):
        return self._calc_block.compute_actual(self._M_target_chain, self._M_camera_chain, self._M_cam)

    def view_error(self):
        import matplotlib.pyplot as plt
        expected = self.compute_expected();
        actual = self.compute_actual();
        plt.plot(expected[0,:].T, expected[1,:].T, 'ro-',
                 actual[0,:].T,   actual[1,:].T, 'bo-',
                 numpy.concatenate([expected[0,:], actual[0,:]],0), numpy.concatenate([expected[1,:], actual[1,:]],0), 'g:')

class CameraChainCalcBlock:
    def update_config(self, full_camera_chain, full_target_chain, camera, cb):
        self._full_camera_chain = full_camera_chain
        self._full_target_chain = full_target_chain
        self._camera = camera
        self._cb = cb

    # Compute the pose of the target, relative to the camera
    def _compute_target_relative_pose(self, camera_chain_state, target_chain_state):

        # Camera pose in root frame
        camera_pose_root = self._full_camera_chain.fk(camera_chain_state)

        # Target pose in root frame
        target_pose_root = self._full_target_chain.fk(target_chain_state)

        # Target pose in camera frame
        target_pose_cam  = camera_pose_root.I * target_pose_root

        return target_pose_cam

    def project_target_to_cam(self, camera_chain_state, target_chain_state, P_list):

        # Get the target's model points in the frame of the tip of the target chain
        target_pts_tip = self._cb.generate_points()

        # Get the pose of target relative to the cam
        target_pose_cam = self._compute_target_relative_pose(camera_chain_state, target_chain_state)

        # Transform the target points into the camera's frame
        target_pts_cam = target_pose_cam * target_pts_tip

        # Do the camera projection
        pixel_pts = self._camera.project(P_list, target_pts_cam)

        return pixel_pts

    def compute_expected(self, cam_M):
        camera_pix = numpy.matrix(sum([[pt.x, pt.y] for pt in cam_M.image_points], []))
        #import code; code.interact(local=locals())
        camera_pix = numpy.reshape(camera_pix, [-1,2])
        camera_pix = camera_pix.T
        return camera_pix

    def compute_actual(self, target_chain_M, camera_chain_M, cam_M):
        return self.project_target_to_cam(camera_chain_M.chain_state,
                                                 target_chain_M.chain_state,
                                                 cam_M.cam_info.P)

    def compute_error(self, target_chain_M, camera_chain_M, cam_M):
        N = len(cam_M.image_points)
        target_pix = self.compute_actual(target_chain_M, camera_chain_M, cam_M)

        assert(target_pix.shape[1] == N)

        camera_pix = self.compute_expected(cam_M)

        error_mat = target_pix - camera_pix

        return error_mat




