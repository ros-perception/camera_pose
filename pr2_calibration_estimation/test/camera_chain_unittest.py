#!/usr/bin/env python
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


import roslib; roslib.load_manifest('pr2_calibration_estimation')

import sys
import unittest
import rospy
import numpy
import yaml
from pr2_calibration_estimation.blocks.camera_chain import CameraChainBundler
from pr2_calibration_estimation.blocks.camera_chain import CameraChainCalcBlock
from pr2_calibration_estimation.blocks.camera_chain import CameraChainRobotParamsBlock

from calibration_msgs.msg import *
from sensor_msgs.msg import JointState, CameraInfo

from pr2_calibration_estimation.single_transform import SingleTransform
from pr2_calibration_estimation.dh_chain import DhChain
from pr2_calibration_estimation.camera import RectifiedCamera
from pr2_calibration_estimation.tilting_laser import TiltingLaser
from pr2_calibration_estimation.full_chain import FullChainCalcBlock
from pr2_calibration_estimation.checkerboard import Checkerboard

from numpy import *

def loadConfigList():

    config_yaml = '''
- block_id: block1
  camera_id: camA
  camera_chain:
    before_chain: [transformA]
    chain_id:     chainA
    dh_link_num:  1
    after_chain:  [transformB]
  target_chain:
    before_chain: [transformC]
    chain_id:     chainB
    after_chain:  [transformD]
    dh_link_num:  6
  error_scalar: 1.0
- block_id: block2
  camera_id: camB
  camera_chain:
    before_chain: [transformA]
    chain_id:     chainA
    dh_link_num:  1
    after_chain:  [transformB]
  target_chain:
    before_chain: [transformC]
    chain_id:     chainB
    after_chain:  [transformD]
    dh_link_num:  6
  error_scalar: 1.0
'''
    config_dict = yaml.load(config_yaml)

    #import code; code.interact(local=locals())
    return config_dict

class TestCameraChainBundler(unittest.TestCase):
    def test_basic_match(self):
        config_list = loadConfigList()

        bundler = CameraChainBundler(config_list)

        M_robot = RobotMeasurement( target_id = "targetA",
                                    chain_id = "chainB",
                                    M_cam   = [ CameraMeasurement(camera_id="camA")],
                                    M_chain = [ ChainMeasurement(chain_id="chainA"),
                                                ChainMeasurement(chain_id="chainB") ])

        blocks = bundler.build_blocks(M_robot)

        self.assertEqual( len(blocks), 1)
        block = blocks[0]
        self.assertEqual( block._M_cam.camera_id,  "camA"  )
        self.assertEqual( block._M_camera_chain.chain_id, "chainA")
        self.assertEqual( block._M_target_chain.chain_id, "chainB")
        self.assertEqual( block._target_id, "targetA")

    def test_basic_no_match(self):
        config_list = loadConfigList()

        bundler = CameraChainBundler(config_list)

        M_robot = RobotMeasurement( target_id = "targetA",
                                    chain_id = "chainA",
                                    M_cam   = [ CameraMeasurement(camera_id="camA")],
                                    M_chain = [ ChainMeasurement(chain_id="chainA"),
                                                ChainMeasurement(chain_id="chainB") ])

        blocks = bundler.build_blocks(M_robot)

        self.assertEqual( len(blocks), 0)


from pr2_calibration_estimation.robot_params import RobotParams

class TestCameraChainRobotParamsBlock(unittest.TestCase):
    def load(self):
        config = yaml.load('''
            block_id: block1
            camera_id: camA
            camera_chain:
              before_chain: [transformA]
              chain_id:     chainA
              dh_link_num:  1
              after_chain:  [transformB]
            target_chain:
              before_chain: [transformC]
              chain_id:     chainB
              after_chain:  [transformD]
              dh_link_num:  6
            ''')

        robot_params = RobotParams()
        robot_params.configure( yaml.load('''
            dh_chains:
              chainA:
              - [ 1, 0, 0, 0 ]
              chainB:
              - [ 2, 0, 0, 0 ]
            tilting_lasers: {}
            rectified_cams:
              camA:
                baseline_shift: 10.0
            transforms:
                transformA: [0, 0, 0, 0, 0, 0]
                transformB: [0, 0, 0, 0, 0, 0]
                transformC: [0, 0, 0, 0, 0, 0]
                transformD: [0, 0, 0, 0, 0, 0]
            checkerboards:
              boardA:
                corners_x: 3
                corners_y: 4
                spacing_x: 10
                spacing_y: 20
            ''' ) )
        return config, robot_params

    def test_update(self):
        config, robot_params = self.load()
        block = CameraChainRobotParamsBlock(config,
                                            CameraMeasurement(camera_id="camA"),
                                            ChainMeasurement(chain_id="chainA"),
                                            ChainMeasurement(chain_id="chainB"),
                                            "boardA")
        block.update_config(robot_params)

        self.assertAlmostEqual(block._calc_block._full_camera_chain._chain._config[0,0], 1, 6)
        #self.assertEqual(block._full_target_chain.calc_block._chain_id, "chainB")
        self.assertAlmostEqual(block._calc_block._camera._baseline_shift, 10, 6)


def load_calc_block():
    calc_block = CameraChainCalcBlock()

    r = 2 / ( 3 * numpy.sqrt(3) ) * numpy.pi

    full_camera_chain = FullChainCalcBlock()
    full_camera_chain.update_config([],
                                    DhChain( [ [0, 0, 1, 0],
                                               [0, 0, 1, 0]] ),
                                    1,
                                    [ SingleTransform([0, 0, 0, -r, r, -r ]) ]) # x up, y left, z backwards

    full_target_chain = FullChainCalcBlock()
    full_target_chain.update_config([SingleTransform([4, 0, 0, 0, -pi/2, 0]) ],
                                     DhChain( [ [0, 0, 1, 0] ]),
                                     0,
                                     [ SingleTransform([0, 0, 0, 0, 0, 0 ]) ])

    cam = RectifiedCamera()

    cb = Checkerboard( { "corners_x": 2,
                         "corners_y": 2,
                         "spacing_x": 1.0,
                         "spacing_y": 1.0 } )

    calc_block.update_config(full_camera_chain, full_target_chain, cam, cb)

    P = [ 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1, 0 ]

    return calc_block, P


class TestCameraChainCalcBlock(unittest.TestCase):
    def test_relative_pose(self):
        calc_block, P_list = load_calc_block()

        camera_chain_state = JointState(position=[-pi/2, pi/2])
        target_chain_state = JointState(position=[-pi/2])

        result = calc_block._compute_target_relative_pose(camera_chain_state, target_chain_state)

        expected = matrix( [ [ 1, 0, 0, 0],
                             [ 0,-1, 0, 0],
                             [ 0, 0,-1, 3],
                             [ 0, 0, 0, 1]] )

        self.assertAlmostEqual(numpy.linalg.norm(result-expected), 0.0, 6)

    def test_project_target_to_cam(self):
        print ""
        calc_block, P_list = load_calc_block()

        camera_chain_state = JointState(position=[-pi/2, pi/2])
        target_chain_state = JointState(position=[-pi/2])

        result = calc_block.project_target_to_cam(camera_chain_state, target_chain_state, P_list)

        print "Result:"
        print result

        expected = matrix( [ [ 0, 1/3.0,    0,  1/3.0 ],
                             [ 0,   0, -1/3.0, -1/3.0 ] ])

        self.assertAlmostEqual(numpy.linalg.norm(result-expected), 0.0, 6)

    def test_compute_error(self):
        print ""
        calc_block, P_list = load_calc_block()

        camera_chain_state = JointState(position=[-pi/2, pi/2])
        target_chain_state = JointState(position=[-pi/2])
        camera_points      = [ ImagePoint(1, 1),
                               ImagePoint(1/3.0, 0),
                               ImagePoint(0, -1/3.0),
                               ImagePoint(1/3.0, -1/3.0) ]


        cam_M = CameraMeasurement(cam_info=CameraInfo(P=P_list),
                                  image_points = camera_points)

        camera_chain_M = ChainMeasurement(chain_state = camera_chain_state )
        target_chain_M = ChainMeasurement(chain_state = target_chain_state )

        result = calc_block.compute_error(target_chain_M, camera_chain_M, cam_M)

        print "Result:"
        print result

        expected = matrix( [ [ -1, 0, 0, 0 ],
                             [ -1, 0, 0, 0 ] ])

        self.assertAlmostEqual(numpy.linalg.norm(result-expected), 0.0, 6)

if __name__ == '__main__':
    import rostest
    rostest.unitrun('pr2_calibration_estimation', 'test_CameraChainBundler',          TestCameraChainBundler,          coverage_packages=['pr2_calibration_estimation.blocks.camera_chain'])
    rostest.unitrun('pr2_calibration_estimation', 'test_CameraChainRobotParamsBlock', TestCameraChainRobotParamsBlock, coverage_packages=['pr2_calibration_estimation.blocks.camera_chain'])
    rostest.unitrun('pr2_calibration_estimation', 'test_CameraChainCalcBlock',        TestCameraChainCalcBlock,        coverage_packages=['pr2_calibration_estimation.blocks.camera_chain'])
