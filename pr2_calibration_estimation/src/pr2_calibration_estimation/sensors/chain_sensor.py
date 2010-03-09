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
from numpy import reshape
import roslib; roslib.load_manifest('pr2_calibration_estimation')
import rospy
from pr2_calibration_estimation.full_chain import FullChainRobotParams

class ChainBundler:
    def __init__(self, valid_configs):
        self._valid_configs = valid_configs

    # Construct a CameraChainRobotParamsBlock for every 'valid config' that finds everything it needs in the current robot measurement
    def build_blocks(self, M_robot):
        sensors = []
        #import code; code.interact(local=locals())
        for cur_config in self._valid_configs:
            cur_chain_id = cur_config["chain_id"]
            if cur_chain_id == M_robot.chain_id and \
               cur_chain_id in [ x.chain_id  for x in M_robot.M_chain ] :
                rospy.logdebug("  Found block")
                M_chain = [x for x in M_robot.M_chain if cur_chain_id == x.chain_id][0]
                cur_sensor = ChainSensor(cur_config, M_chain, M_robot.target_id)
                sensors.append(cur_sensor)
            else:
                rospy.logdebug("  Didn't find block")
        return sensors

class ChainSensor:
    def __init__(self, config_dict, M_chain, target_id):

        self.sensor_type = "chain"
        self.sensor_id   = config_dict["chain_id"]

        self._config_dict = config_dict
        self._M_chain = M_chain
        self._target_id = target_id

        self._full_chain = FullChainRobotParams(self._config_dict)

    def update_config(self, robot_params):
        self._full_chain.update_config(robot_params)
        self._checkerboard = robot_params.checkerboards[ self._target_id ]

    def compute_residual(self, target_pts):
        h_mat = self.compute_expected(target_pts)
        z_mat = self.get_measurement()
        assert(h_mat.shape == z_mat.shape)
        assert(h_mat.shape[0] == 4)
        r_mat = h_mat[0:3,:] - z_mat[0:3,:]
        r = reshape(r_mat.T, [-1,1])
        return r

    def get_measurement(self):
        # Get the target's model points in the frame of the tip of the target chain
        target_pts_tip = self._checkerboard.generate_points()

        # Target pose in root frame
        target_pose_root = self._full_chain.calc_block.fk(self._M_chain.chain_state)

        # Transform points into the root frame
        target_pts_root = target_pose_root * target_pts_tip

        return target_pts_root

    def compute_expected(self, target_pts):
        return target_pts

    # Build a dictionary that defines which parameters will in fact affect this measurement
    def build_sparsity_dict(self):
        sparsity = dict()
        sparsity['transforms'] = {}
        for cur_transform_name in ( self._config_dict['before_chain'] + self._config_dict['after_chain'] ):
            sparsity['transforms'][cur_transform_name] = [1, 1, 1, 1, 1, 1]

        sparsity['dh_chains'] = {}
        chain_id = self._config_dict['chain_id']
        num_links = self._full_chain.calc_block._chain._M
        assert(num_links == len(self._M_chain.chain_state.position))
        sparsity['dh_chains'][chain_id] = [ [1,1,1,1] ] * num_links

        return sparsity

