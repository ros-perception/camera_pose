#! /usr/bin/env python
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

import roslib; roslib.load_manifest('pr2_calibration_estimation')

import sys
import rospy
import time
import numpy
import rosrecord
import yaml

import numpy

from pr2_calibration_estimation.robot_params import RobotParams
from pr2_calibration_estimation.blocks import robot_measurement_bundler

def usage():
    print "Usage:"
    print " ./error_viewer.py [config.yaml] [blocks.yaml] [bagfile]"
    sys.exit(-1)

if (len(sys.argv) < 4):
    usage()

config_filename = sys.argv[1]
blocks_filename= sys.argv[2]
bag_filename = sys.argv[3]

# Load the yaml config of the robot_system
config_f = open(config_filename)
config = yaml.load(config_f)
config_f.close()
robot_params = RobotParams()
robot_params.configure(config)

# Load the measurement blocks
blocks_f = open(blocks_filename)
measurement_blocks = yaml.load(blocks_f)
blocks_f.close()
bundler = robot_measurement_bundler.RobotMeasurementBundler(measurement_blocks)

blocks = bundler.load_from_bag(bag_filename)


for block in blocks:
    block.update_config(robot_params)

from pr2_calibration_estimation import plot_helper
plot_helper.start()

from matplotlib import pyplot as plt

for block in blocks:
    resp = raw_input("> ").upper()
    print "Type: %s" % block.block_type
    print "ID: %s" % block.block_id
    plt.clf()
    block.view_error()
    plt.axis([0, 640, 0, 480])
    print "About to draw"
    plt.draw()
    print "Done drawing"
    #time.sleep(.1)


sys.exit(1)
