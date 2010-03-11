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

from pr2_calibration_estimation.robot_params import RobotParams
from pr2_calibration_estimation.single_transform import SingleTransform
import numpy
import scipy.optimize
import sys

class ErrorCalc:
    """
    Helpers for computing errors and jacobians
    """
    def __init__(self, robot_params, free_dict, multisensors):
        self._robot_params = robot_params
        self._expanded_params = robot_params.deflate()
        self._free_list = robot_params.calc_free(free_dict)
        self._multisensors = multisensors


    def calculate_full_param_vec(self, opt_param_vec):
        '''
        Take the set of optimization params, and expand it into the bigger param vec
        '''
        full_param_vec = self._expanded_params.copy()
        full_param_vec[numpy.where(self._free_list), 0] = opt_param_vec

        return full_param_vec

    def calculate_error(self, opt_all_vec):
        # print "x ",
        # sys.stdout.flush()

        opt_param_len = self._expanded_params.shape[0]
        opt_param_vec = opt_all_vec[0:opt_param_len]
        full_pose_vec  = opt_all_vec[opt_param_len:]

        full_param_vec = self.calculate_full_param_vec(opt_param_vec)

        full_pose_arr = numpy.reshape(full_pose_vec, [-1,6])

        # Update the primitives with the new set of parameters
        self._robot_params.inflate(full_param_vec)

        # Update all the blocks' configs (This might not be necessary, because everything should point to the correct object
        for multisensor in self._multisensors:
            for sensor in multisensor.sensors:
                sensor.update_config(self._robot_params)

        for multisensor, cb_pose in zip(self._multisensors, list(full_pose_arr)):
            # Process cb pose
            cb_T = SingleTransform(cb_pose).transform
            cb_model = self._robot_params.checkerboards[multisensor.checkerboard]
            cb_points = cb_T * cb_model.generate_points()
            r = [sensor.compute_residual(cb_points) for sensor in multisensor]


        # Compute Errors
        errors = [numpy.reshape(block.compute_residual(), (-1,1)) for block in self._blocks]

        error_vec = numpy.concatenate(errors, 0)

        error_array = numpy.array(error_vec).T[0]

        rms_error = numpy.sqrt( numpy.mean(error_array**2) )
        print "%.3f " % rms_error,
        sys.stdout.flush()

        return error_array.copy()

    def calculate_jacobian(self, opt_param_vec):
        import scipy.optimize.slsqp.approx_jacobian as approx_jacobian
        J = approx_jacobian(opt_param_vec, self.calculate_error, 1e-6)
        return J

    def single_sensor_params_jacobian(self, opt_param_vec, target_points, sensor):
        sparsity_dict = sensor.build_sparsity_dict()
        required_keys = ['dh_chains', 'tilting_lasers', 'transforms', 'rectified_cams', 'checkerboards']
        for cur_key in required_keys:
            if cur_key not in sparsity_dict.keys():
                sparsity_dict[cur_key] = {}
        # Generate the full sparsity vector
        full_sparsity_list = self._robot_params.calc_free(sparsity_dict)
        full_sparsity_vec = numpy.array(full_sparsity_list)

        # Extract the sparsity for only the parameters we are optimizing over

        #import code; code.interact(local=locals())
        opt_sparsity_vec = full_sparsity_vec[numpy.where(self._free_list)].copy()

        # Update the primitives with the new set of parameters
        full_param_vec = self.calculate_full_param_vec(opt_param_vec)
        self._robot_params.inflate(full_param_vec)
        sensor.update_config(self._robot_params)

        # based on code from scipy.slsqp
        x0 = opt_param_vec
        epsilon = 1e-6
        f0 = sensor.compute_residual(target_points)
        Jt = numpy.zeros([len(x0),len(f0)])
        dx = numpy.zeros(len(x0))
        for i in numpy.where(opt_sparsity_vec)[0]:
            dx[i] = epsilon
            opt_test_param_vec = x0 + dx
            full_test_param_vec = self.calculate_full_param_vec(opt_test_param_vec)
            self._robot_params.inflate(full_test_param_vec)
            sensor.update_config(self._robot_params)
            #import code; code.interact(local=locals())
            Jt[i] = numpy.array(((sensor.compute_residual(target_points) - f0)/epsilon).T)[0]
            dx[i] = 0.0
        J = Jt.transpose()
        return J

def opt_runner(robot_params_dict, free_dict, blocks, pose_guesses):
    """
    Runs a single optimization step for the calibration optimization.
      robot_params_dict - Dictionary storing all of the system primitives' parameters (lasers, cameras, chains, transforms, etc)
      free_dict - Dictionary storing which parameters are free
      blocks - list of list of measurements. Each sublist corresponds to a single checkerboard pose
      pose_guesses - List of guesses as to where all the checkerboard are. This is used to initialze the optimization
    """
    # Load the robot params
    robot_params = RobotParams()
    robot_params.configure(robot_params_dict)

    # Determine how many checkerboard poses we need to estimate
    num_poses = len(blocks)
    assert(num_poses == len(pose_guesses))

    for cur_block_list in blocks:
        for block in cur_block_list:
            block.update_config(robot_params)

    error_calc = ErrorCalc(robot_params, free_dict, blocks)

    # Construct the initial guess
    expanded_param_vec = robot_params.deflate()
    free_list = robot_params.calc_free(free_dict)
    opt_param_mat = expanded_param_vec[numpy.where(free_list), 0].copy()
    opt_param = numpy.array(opt_param_mat)[0]

    opt_pose_mat = numpy.matrix(pose_guesses)
    assert(opt_pose_mat.shape[1] == 6)
    opt_pose = numpy.array(numpy.reshape(opt_pose_mat, [1,-1]))[0]

    opt_all = numpy.concatenate([opt_param, opt_pose])

    x, cov_x, infodict, mesg, iter = scipy.optimize.leastsq(error_calc.calculate_error, opt_all, Dfun=error_calc.calculate_jacobian, full_output=1)

    # A hacky way to inflate x back into robot params
    full_param_vec = error_calc.calculate_full_param_vec(x)

    output_dict = error_calc._robot_params.params_to_config(full_param_vec)

    # Compute the rms error
    final_error = error_calc.calculate_error(x)
    rms_error = numpy.sqrt( numpy.mean(final_error**2) )
    print "RMS Error: %f" % rms_error

    return output_dict














