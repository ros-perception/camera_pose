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
#       pre_chain_Ts -- camera_chain -- pos_chain_Ts -- camera
#      /
#   root
#      \
#       tilting_laser

import numpy
from numpy import matrix
import rospy
from pr2_calibration_estimation.full_chain import FullChainRobotParams

# Takes a measurment, plus a set of possible camera/laser pairs, and creates the necessary error blocks
class CameraLaserBundler:
    def __init__(self, valid_configs):
        self._valid_configs = valid_configs

    # Construct a CameraLaserErrorBlock for each camera/laser pair that matches one of the configs
    def build_blocks(self, M_robot):

        blocks = []
        for cur_config in self._valid_configs:
            rospy.logdebug("On Config Block [%s]-[%s]" % (cur_config["camera_id"], cur_config["laser_id"]) )
            if cur_config["camera_id"] in [ x.camera_id for x in M_robot.M_cam ] and \
               cur_config["laser_id"]  in [ x.laser_id  for x in M_robot.M_laser ] and \
               cur_config["full_camera_chain"]["chain_id"]  in [ x.chain_id  for x in M_robot.M_chain ]:
                rospy.logdebug("  Found block!")
                cur_M_cam   = M_robot.M_cam  [ [ x.camera_id for x in M_robot.M_cam   ].index(cur_config["camera_id"])]
                cur_M_laser = M_robot.M_laser[ [ x.laser_id  for x in M_robot.M_laser ].index(cur_config["laser_id"]) ]
                cur_M_chain = M_robot.M_chain[ [ x.chain_id  for x in M_robot.M_chain ].index(cur_config["full_camera_chain"]["chain_id"]) ]
                cur_block = CameraLaserRobotParamsBlock(cur_M_cam, cur_M_laser, cur_M_chain, cur_config)
                cur_block.error_scalar = cur_config["error_scalar"]
                blocks.append(cur_block)
            else:
                rospy.logdebug("  Didn't find block")
        return blocks

class CameraLaserRobotParamsBlock:
    def __init__(self, M_cam, M_laser, M_chain, config_dict):
        self._config_dict = config_dict
        self.block_type = "CameraLaser"
        self.block_id = config_dict["block_id"]

        self._M_cam = M_cam
        self._M_laser = M_laser
        self._M_chain = M_chain

        #print "P"
        #print self._M_cam.cam_info.P
        #print self._M_cam.cam_info.height
        #print self._M_cam.cam_info.width

        self._full_chain = FullChainRobotParams(config_dict["full_camera_chain"])
        self._calc_block = CameraLaserCalcBlock()

    def update_config(self, robot_params):
        camera            = robot_params.rectified_cams[ self._config_dict["camera_id"] ]
        tilting_laser     = robot_params.tilting_lasers[ self._config_dict["laser_id"] ]

        self._full_chain.update_config(robot_params)
        self._calc_block.update_config(self._full_chain.calc_block, camera, tilting_laser)

    def compute_error(self):
        error_mat = self._calc_block.compute_error(self._M_chain, self._M_cam, self._M_laser)
        #error_vec = numpy.reshape(error_mat, (-1,1))
        #return error_vec
        return error_mat

    # Returns the pixel coordinates as sensed by the camera
    def compute_expected(self):
        return self._calc_block.compute_expected(self._M_cam)

    # Returns the pixel coordinates of the laser points after being projected into the camera
    def compute_actual(self):
        return self._calc_block.compute_actual(self._M_chain, self._M_cam, self._M_laser)

    def get_error_length(self):
        N = len(self._M_laser.joint_points.joint_points)
        return 2*N

    def view_error(self):
        import matplotlib.pyplot as plt
        expected = self.compute_expected();
        actual = self.compute_actual();
        plt.plot(expected[0,:].T, expected[1,:].T, 'ro-',
                 actual[0,:].T,   actual[1,:].T, 'bo-',
                 numpy.concatenate([expected[0,:], actual[0,:]],0), numpy.concatenate([expected[1,:], actual[1,:]],0), 'g:')


class CameraLaserCalcBlock:

    # Pass in pointers to the kinematics and sensor blocks that we need
    def update_config(self, full_cam_chain, camera, tilting_laser):
        self._full_cam_chain = full_cam_chain
        self._camera         = camera
        self._tilting_laser  = tilting_laser

    # Given a laser measurement, project it into image coordinates. Returns a 2xN Matrix
    def _project_laser_to_cam(self, chain_state, P_list, laser_joint_points):
        # Get the laser points in a 4xN homogenous matrix
        laser_pts_root = self._tilting_laser.project_to_3D([x.position for x in laser_joint_points])

        # print "Laser Points Root:"
        # print laser_pts_root

        #import code; code.interact(local=locals())

        # Compute the pose of the camera
        cam_pose = self._full_cam_chain.fk(chain_state)

        # Transform laser points into camera frame
        laser_pts_cam = cam_pose.I * laser_pts_root

        # print "Laser Points Cam:"
        # print laser_pts_cam

        # Project points into camera
        pixel_points = self._camera.project(P_list, laser_pts_cam)

        return pixel_points

    # Returns 2xN matrix with pixel coordinates as sensed by the camera
    def compute_expected(self, cam_M):
        camera_pix = numpy.matrix(sum([[pt.x, pt.y] for pt in cam_M.image_points], []))
        camera_pix.shape = (-1,2)
        camera_pix = camera_pix.T
        return camera_pix

    # Returns 2xN matrix with pixel coordinates of laser points projected into camera coordinates
    def compute_actual(self, chain_M, cam_M, laser_M):
        return self._project_laser_to_cam(chain_M.chain_state, cam_M.cam_info.P, laser_M.joint_points)

    # Computes the pixel error between laser points projected into the camera, and the sensed camera points
    # Returns 2xN matrix with pixel errors
    def compute_error(self, chain_M, cam_M, laser_M):
        N = len(laser_M.joint_points)
        laser_pix = self.compute_actual(chain_M, cam_M, laser_M)

        #import code; code.interact(local=locals())

        assert(len(cam_M.image_points) == N)
        camera_pix = self.compute_expected(cam_M)

        # print "Laser pix:"
        # print laser_pix

        # print "\nCamera Pix:"
        # print camera_pix

        # Compute pixel difference between laser and camera data
        error_mat = laser_pix - camera_pix

        # import code; code.interact(local=locals())

        return error_mat
