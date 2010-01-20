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
import time
import numpy

from pr2_calibration_estimation.camera import RectifiedCamera
from numpy import *

class TestRectifiedCamera(unittest.TestCase):

    def test_project_easy_1(self):
        cam = RectifiedCamera({"baseline_shift":0})
        P_list = [ 1, 0, 0, 0, \
                   0, 1, 0, 0, \
                   0, 0, 1, 0 ]

        pts = matrix( [ [ 0, 1, 0, 2 ],
                        [ 0, 1, 1, 0 ],
                        [ 1, 1, 2, 2 ],
                        [ 1, 1, 1, 1 ]], float )

        expected = matrix( [ [ 0, 1,  0, 1],
                             [ 0, 1, .5, 0] ] )

        result = cam.project(P_list, pts)

        print ""
        print result

        self.assertAlmostEqual(numpy.linalg.norm(result-expected), 0.0, 6)

    def test_project_easy_2(self):
        cam = RectifiedCamera({"baseline_shift":0})
        P_list = [ 1, 0, 0,-1, \
                   0, 1, 0, 0, \
                   0, 0, 1, 0 ]

        pts = matrix( [ [ 0, 1, 0, 2 ],
                        [ 0, 1, 1, 0 ],
                        [ 1, 1, 2, 2 ],
                        [ 1, 1, 1, 1 ]], float )

        expected = matrix( [ [-1, 0,-.5,.5],
                             [ 0, 1, .5, 0] ] )

        result = cam.project(P_list, pts)

        print ""
        print result

        self.assertAlmostEqual(numpy.linalg.norm(result-expected), 0.0, 6)

    def test_project_easy_3(self):
        cam = RectifiedCamera({"baseline_shift":0})
        P_list = [ 1, 0, 0, 0, \
                   0, 1, 0, 0, \
                   0, 0, 1, 0 ]

        cam.inflate(matrix([-1]))

        pts = matrix( [ [ 0, 1, 0, 2 ],
                        [ 0, 1, 1, 0 ],
                        [ 1, 1, 2, 2 ],
                        [ 1, 1, 1, 1 ]], float )

        expected = matrix( [ [-1, 0,-.5,.5],
                             [ 0, 1, .5, 0] ] )

        result = cam.project(P_list, pts)

        print ""
        print result

        self.assertAlmostEqual(numpy.linalg.norm(result-expected), 0.0, 6)


    def test_free(self):
        cam = RectifiedCamera({"baseline_shift":0})
        free_list = cam.calc_free( {"baseline_shift":0} )
        self.assertEqual(free_list[0], False)

        free_list = cam.calc_free( {"baseline_shift":1} )
        self.assertEqual(free_list[0], True)

        self.assertEqual(len(free_list), 1)

    def test_params_to_config(self):
        cam = RectifiedCamera()
        p = matrix([1], float)
        config = cam.params_to_config(p)
        self.assertAlmostEqual(config["baseline_shift"], 1, 6)

    def test_length(self):
        cam = RectifiedCamera({"baseline_shift":0})
        self.assertEqual(cam.get_length(), 1)

    def test_deflate(self):
        cam = RectifiedCamera({"baseline_shift":10})
        self.assertEqual(cam.deflate()[0,0], 10)

if __name__ == '__main__':
    import rostest
    rostest.unitrun('pr2_calibration_estimation', 'test_RectifiedCamera', TestRectifiedCamera, coverage_packages=['pr2_calibration_estimation.camera'])

