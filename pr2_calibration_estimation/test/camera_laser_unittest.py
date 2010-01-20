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
from pr2_calibration_estimation.blocks.camera_laser import CameraLaserBundler
from pr2_calibration_estimation.blocks.camera_laser import CameraLaserCalcBlock
from calibration_msgs.msg import *
from sensor_msgs.msg import JointState, CameraInfo

from pr2_calibration_estimation.single_transform import SingleTransform
from pr2_calibration_estimation.dh_chain import DhChain
from pr2_calibration_estimation.camera import RectifiedCamera
from pr2_calibration_estimation.tilting_laser import TiltingLaser
from pr2_calibration_estimation.full_chain import FullChainCalcBlock

from numpy import *

def loadConfigList():

    config_yaml = '''
- block_id:     block1
  camera_id:    cam1
  laser_id:     laser1
  full_camera_chain:
    chain_id:     chain1
  error_scalar: 1.0

- block_id:     block2
  camera_id:    cam2
  laser_id:     laser1
  full_camera_chain:
    chain_id:     chain1
  error_scalar: 1.0

- block_id:     block3
  camera_id:    cam2
  laser_id:     laser1
  full_camera_chain:
    chain_id:     chain2
  error_scalar: 1.0
'''
    config_dict = yaml.load(config_yaml)

    #import code; code.interact(local=locals())
    return config_dict

class TestCameraLaserBundler(unittest.TestCase):
    def test_basic(self):
        config_list = loadConfigList()

        bundler = CameraLaserBundler(config_list)

        M_robot = RobotMeasurement()
        M_robot.M_cam.append(  CameraMeasurement(camera_id="cam1"))
        M_robot.M_cam.append(  CameraMeasurement(camera_id="cam2"))
        M_robot.M_laser.append( LaserMeasurement(laser_id="laser1"))
        M_robot.M_chain.append( ChainMeasurement(chain_id="chain1"))

        blocks = bundler.build_blocks(M_robot)

        self.assertEqual( len(blocks), 2)
        self.assertEqual( blocks[0]._M_cam.camera_id,  "cam1"  )
        self.assertEqual( blocks[0]._M_laser.laser_id, "laser1")
        self.assertEqual( blocks[0]._M_chain.chain_id, "chain1")

        self.assertEqual( blocks[1]._M_cam.camera_id,  "cam2"  )
        self.assertEqual( blocks[1]._M_laser.laser_id, "laser1")
        self.assertEqual( blocks[1]._M_chain.chain_id, "chain1")


def loadSystem1():
    calc_block = CameraLaserCalcBlock()
    before_chain_Ts = [SingleTransform([10, 0, 0, 0, 0, 0])]
    chain = DhChain( [ [0, 0, 1, 0] ])
    dh_link_num = -1
    after_chain_Ts = [SingleTransform([ 0, 0, 20, 0, 0, 0])]

    cam = RectifiedCamera()
    tilt_laser = TiltingLaser()

    full_cam_chain = FullChainCalcBlock()
    full_cam_chain.update_config(before_chain_Ts, chain, dh_link_num, after_chain_Ts)
    calc_block.update_config(full_cam_chain, cam, tilt_laser)
    return calc_block

def loadSystem2():
    calc_block = CameraLaserCalcBlock()
    before_chain_Ts = []
    chain = DhChain( [ [0, 0, 1, 0],
                       [0, 0, 1, 0]] )
    dh_link_num = -1
    r = 2 / ( 3 * numpy.sqrt(3) ) * numpy.pi
    after_chain_Ts = [ SingleTransform([0, 0, 0, -r, r, -r ]) ]

    cam = RectifiedCamera()
    tilt_laser = TiltingLaser({"before_joint": [  3, 0, 0, 0, 0, 0],
                               "after_joint" : [  0, 0, 0, 0, 0, 0] })

    full_cam_chain = FullChainCalcBlock()
    full_cam_chain.update_config(before_chain_Ts, chain, dh_link_num, after_chain_Ts)
    calc_block.update_config(full_cam_chain, cam, tilt_laser)

    P = [ 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1, 0 ]

    return calc_block, P


class TestCameraLaserCalcBlock_Easy(unittest.TestCase):

    def test_laser_to_cam_1(self):
        print ""
        calc_block, P = loadSystem2()

        chain_state = JointState(position=[-pi/2, pi/2])

        joint_points = [ JointState(position=[0,0,0]),
                         JointState(position=[0,pi/2,1]),
                         JointState(position=[pi/2,0,1]) ]

        result = calc_block._project_laser_to_cam(chain_state, P, joint_points)

        print "Result"
        print result

        expected = numpy.matrix( [[ -.5, -1, -.5 ],
                                  [   0,  0,  .5 ]] )

        self.assertAlmostEqual(numpy.linalg.norm(result-expected), 0.0, 6)

    def test_compute_error(self):
        print ""
        calc_block, P = loadSystem2()
        camera_points = [ImagePoint(x = 1, y = 1),
                         ImagePoint(x = 0, y = 1),
                         ImagePoint(x = 0, y = 0)]

        chain_state = JointState(position=[-pi/2, pi/2])
        joint_points = [ JointState(position=[0,0,0]),
                         JointState(position=[0,pi/2,1]),
                         JointState(position=[pi/2,0,1]) ]

        chain_M = ChainMeasurement(chain_state=chain_state)
        cam_M   = CameraMeasurement(cam_info=CameraInfo(P=P), image_points=camera_points)
        laser_M = LaserMeasurement(joint_points=joint_points)

        result = calc_block.compute_error(chain_M, cam_M, laser_M)
        print "Result:"
        print result

        expected = matrix( [[ -1.5, -1,-.5],
                            [  -1, -1,  .5]])


        self.assertAlmostEqual(numpy.linalg.norm(result-expected), 0.0, 6)

if __name__ == '__main__':
    import rostest
    rostest.unitrun('pr2_calibration_estimation', 'test_CameraLaserBundler',   TestCameraLaserBundler,   coverage_packages=['pr2_calibration_estimation.blocks.camera_laser'])
    rostest.unitrun('pr2_calibration_estimation', 'test_CameraLaserCalcBlock_Easy', TestCameraLaserCalcBlock_Easy, coverage_packages=['pr2_calibration_estimation.blocks.camera_laser'])
