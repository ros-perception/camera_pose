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
    rospy.logerr("Not enough arguments")
    print "Usage:"
    print " ./proto1.py [bagfile] [output_dir]"
    sys.exit(0)

class ErrorCalc:
    def __init__(self, robot_params, expanded_params, free_list, blocks):
        self._robot_params = robot_params
        self._expanded_params = expanded_params
        self._free_list = free_list
        self._blocks = blocks

    # Take the set of optimization params, and expand it into the bigger param vec
    def calculate_full_param_vec(self, opt_param_vec):
        full_param_vec = self._expanded_params.copy()

        #import code; code.interact(local=locals())

        full_param_vec[numpy.where(self._free_list), 0] = opt_param_vec

        return full_param_vec

    def calculate_error(self, opt_param_vec):
        # print "x ",
        # sys.stdout.flush()

        full_param_vec = self.calculate_full_param_vec(opt_param_vec)

        # Update the primitives with the new set of parameters
        self._robot_params.inflate(full_param_vec)
        # Update all the blocks' configs (This might not be necessary, because everything should point to the correct object
        for block in self._blocks:
            block.update_config(self._robot_params)

        # Compute Errors
        errors = [numpy.reshape(block.compute_error()*block.error_scalar, (-1,1)) for block in self._blocks]

        error_vec = numpy.concatenate(errors, 0)

        error_array = numpy.array(error_vec).T[0]

        rms_error = numpy.sqrt( numpy.mean(error_array**2) )
        print "%.3f " % rms_error,
        sys.stdout.flush()

        return error_array.copy()

def opt_runner(robot_params_dict, free_dict, blocks):
    # Load the robot params
    robot_params = RobotParams()
    robot_params.configure(robot_params_dict)

    for block in blocks:
        block.update_config(robot_params)

    # Load the free configuration
    free_list = robot_params.calc_free(free_dict)
    expanded_param_vec = robot_params.deflate()

    error_calc = ErrorCalc(robot_params, expanded_param_vec, free_list, blocks)

    # Construct the initial guess
    opt_guess_mat = expanded_param_vec[numpy.where(free_list), 0].copy()
    opt_guess = numpy.array(opt_guess_mat)[0]

    import scipy.optimize
    x, cov_x, infodict, mesg, iter = scipy.optimize.leastsq(error_calc.calculate_error, opt_guess.copy(), full_output=1)

    # A hacky way to inflate x back into robot params
    full_param_vec = error_calc.calculate_full_param_vec(x)

    output_dict = error_calc._robot_params.params_to_config(full_param_vec)

    # Compute the rms error
    final_error = error_calc.calculate_error(x)
    rms_error = numpy.sqrt( numpy.mean(final_error**2) )
    print "RMS Error: %f" % rms_error

    return output_dict


rospy.init_node("robot_cal_estimator")

if (len(rospy.myargv()) < 2):
    usage()
elif (len(rospy.myargv()) < 3):
    bag_filename = rospy.myargv()[1]
    output_dir = "."
else:
    bag_filename = rospy.myargv()[1]
    output_dir = rospy.myargv()[2]

config = rospy.get_param("calibration_config")

print "Keys: %s" % config.keys()

# Specify which system the current calibration step should use.
# Normally this would be set at the end of the calibration loop, but for the first step,
# this is grabbed from the param server
previous_system = yaml.load(config["initial_system"])

# Param server dictionary has no concept of order, but we want to execute steps in name-sorted order
step_keys = config["cal_steps"].keys()
step_keys.sort()

step_list = []
for step_name in step_keys:
    # Add the step name to the dict (since we'll lose this info once we listify)
    config["cal_steps"][step_name]["name"] = step_name
    step_list.append(config["cal_steps"][step_name])

rospy.loginfo("Going to execute steps in the following order: %s" % [x["name"] for x in step_list])

# Load all the blocks from the bagfile and config file

for cur_step in step_list:
    print ""
    print "****** Starting Step [%s] ******" % cur_step["name"]

    bundler = robot_measurement_bundler.RobotMeasurementBundler(yaml.load(cur_step["blocks"]))
    blocks = bundler.load_from_bag(bag_filename)

    free_dict = yaml.load(cur_step["free_params"])
    output_dict = opt_runner(previous_system, free_dict, blocks)

    out_f = open(output_dir + "/" + cur_step["output_filename"], 'w')
    yaml.dump(output_dict, out_f)
    out_f.close()

    previous_system = output_dict

#camera_laser_bundler = camera_laser.CameraLaserBundler( measurement_blocks["camera_laser"] )
#camera_chain_bundler = camera_chain.CameraChainBundler( measurement_blocks["camera_chain"] )

#blocks = []
#for topic, msg, t in rosrecord.logplayer(bag_f):
#    if topic == "robot_measurement":
#        if msg.chain_id == "right_arm":
#            msg.chain_id = "right_arm_chain"
#
#        print "****** On RobotMeasurement message %u ******" % msg_count
#        #print "  CamIDs:   %s" % ", ".join([x.camera_id for x in msg.M_cam])
#        #print "  ChainIDs: %s" % ", ".join([x.chain_id  for x in msg.M_chain])
#        #print "  LaserIDs: %s" % ", ".join([x.laser_id  for x in msg.M_laser])
#        rospy.logdebug( "****** On RobotMeasurement message %u ******" % msg_count)
#        cur_blocks = camera_laser_bundler.build_blocks(msg)
#        print "  Found %u Camera-Laser blocks in msg" % len(cur_blocks)
#        blocks.extend(cur_blocks)
#
#        cur_blocks = camera_chain_bundler.build_blocks(msg)
#        print "  Found %u Camera-Chain blocks in msg" % len(cur_blocks)
#        blocks.extend(cur_blocks)
#
#        msg_count += 1



#error1 = blocks[0].compute_error()
#print "Computing Errors"
#start = time.clock()
#errors = [block.compute_error() for block in blocks]
#end = time.clock()
#print "Done"
#error_vec = numpy.concatentate(errors, 0)

#import code; code.interact(local=locals())
