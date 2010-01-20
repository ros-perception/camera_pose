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

import numpy
from numpy import matrix, vsplit, sin, cos, reshape, ones
import rospy


# Primitive for projecting 3D points into a monocular camera. The baseline
# term of the projection matrix can be changed.  The baseline shift is added
# to the P[0,3] term of the projection matrix found in CameraInfo
class RectifiedCamera:
    # Config - Dictionary with a a single key: "baseline_shift"
    def __init__(self, config={"baseline_shift":0}):
        rospy.logdebug("Initializng rectified camera")
        self._baseline_shift = config["baseline_shift"]

    def calc_free(self, free_config):
        assert( 'baseline_shift' in free_config )
        return [free_config["baseline_shift"] == 1]

    def params_to_config(self, param_vec):
        assert(param_vec.shape == (1,1))
        return {"baseline_shift": float(param_vec[0,0])}

    # Convert column vector of params into config, expects a 1x1 matrix
    def inflate(self, param_vec):
        self._baseline_shift = param_vec[0,0]

    # Return column vector of config. In this case, it's always a 1x1 matrix
    def deflate(self):
        return matrix([self._baseline_shift])

    # Returns # of params needed for inflation & deflation
    def get_length(self):
        return 1

    # Project a set of 3D points points into pixel coordinates
    # P_list - Projection matrix. We expect this to be a 1x12 list. We then reshape
    #          it into a 3x4 matrix (by filling 1 row at a time)
    # pts - 4xN numpy matrix holding the points that we want to project (homogenous coords)
    def project(self, P_list, pts):
        N = pts.shape[1]

        # Reshape P_list into an actual matrix
        P = reshape( matrix(P_list, float), (3,4) )

        # Update the baseline by the "baseline_shift"
        P[0,3] = P[0,3] + self._baseline_shift

        #import code; code.interact(local=locals())
        if (pts.shape[0] == 3):
            rospy.logfatal("Got vector of points with only 3 rows. Was expecting at 4 rows (homogenous coordinates)")


        # Apply projection matrix
        pixel_pts_h = P * pts

        # print "pixel_points_h"
        # print pixel_pts_h

        #import code; code.interact(local=locals())

        # Strip out last row (3rd) and rescale
        pixel_pts = pixel_pts_h[0:2,:] / pixel_pts_h[2,:]

        return pixel_pts

