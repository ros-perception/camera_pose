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

# Loads all the blocks from a robot measurements

import roslib; roslib.load_manifest('pr2_calibration_estimation')
import rosrecord

from pr2_calibration_estimation.blocks import camera_laser
from pr2_calibration_estimation.blocks import camera_chain


class RobotMeasurementBundler:
    def __init__(self, block_configs):
        self._camera_laser_bundler = camera_laser.CameraLaserBundler( block_configs["camera_laser"] )
        self._camera_chain_bundler = camera_chain.CameraChainBundler( block_configs["camera_chain"] )
        #import code; code.interact(local=locals())

    # Read all the measurements that are in a given bagfile
    def load_from_bag(self, bag_filename):

        f = open(bag_filename)
        msg_count = 0
        blocks = []
        for topic, msg, t in rosrecord.logplayer(f):
            if topic == "robot_measurement":
                if msg.chain_id == "right_arm":
                    msg.chain_id = "right_arm_chain"

                #print "****** On RobotMeasurement message %u ******" % msg_count
                cur_blocks = self.load_from_msg(msg)
                blocks.extend(cur_blocks)

                msg_count += 1

        f.close()

        return blocks

    def load_from_msg(self, msg):
        blocks = []
        #print "  CamIDs:   %s" % ", ".join([x.camera_id for x in msg.M_cam])
        #print "  ChainIDs: %s" % ", ".join([x.chain_id  for x in msg.M_chain])
        #print "  LaserIDs: %s" % ", ".join([x.laser_id  for x in msg.M_laser])
        cur_blocks = self._camera_laser_bundler.build_blocks(msg)
        #print "  Found %u Camera-Laser blocks in msg" % len(cur_blocks)
        blocks.extend(cur_blocks)

        cur_blocks = self._camera_chain_bundler.build_blocks(msg)
        #print "  Found %u Camera-Chain blocks in msg" % len(cur_blocks)
        blocks.extend(cur_blocks)

        return blocks


