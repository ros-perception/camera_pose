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

import roslib; roslib.load_manifest('megacal_estimation')
import rospy
import yaml
import sys
import actionlib
import time
import threading

import pr2_calibration_executive
from pr2_calibration_executive.config_manager import ConfigManager
from pr2_calibration_executive.sensor_managers import *
from pr2_calibration_executive.robot_measurement_cache import RobotMeasurementCache

from calibration_msgs.msg import RobotMeasurement

class CameraCaptureExecutive:
    def __init__(self, cam_ids):
        self.cam_ids = cam_ids
        self.cache = RobotMeasurementCache()
        self.lock = threading.Lock()

        # Specifies if we're currently waiting for a sample
        self.active = False

        # Construct a manager for each sensor stream (Don't enable any of them)
        self.cam_managers   = [ (cam_id,   CamManager(  cam_id,   self.add_cam_measurement) )   for cam_id   in cam_ids ]

        # Turn on all of the camera modes into verbose model, since we want the CalibrationPattern data
        for cam_manager in zip(*self.cam_managers)[1]:
            cam_manager.enable(verbose=True)

        # Subscribe to topic containing stable intervals
        self.measurement_pub = rospy.Publisher("robot_measurement", calibration_msgs.msg.RobotMeasurement)
        self.request_interval_sub = rospy.Subscriber("request_interval", calibration_msgs.msg.Interval, self.request_callback)

        # Set up the cache with only the sensors we care about
        chain_ids = []
        laser_ids = []
        self.cache.reconfigure(cam_ids, chain_ids, laser_ids)

    def request_callback(self, msg):

        print "Got a request callback"

        # See if the interval is big enough to care
        #if (msg.end - msg.start) < rospy.Duration(1,0):
        #    return

        self.lock.acquire()
        # Do some query into the cache
        m_robot = self.cache.request_robot_measurement(msg.start, msg.end, min_duration=rospy.Duration(0.01))

        # We found a sample, so we can deactive (kind of a race condition, since 'active' triggers capture() to exit... I don't care)
        if m_robot:
            # Change camera ids to be the tf frame IDs
            for cam in m_robot.M_cam:
                cam.camera_id = cam.image.header.frame_id
            self.measurement_pub.publish(m_robot)
        else:
            print "Couldn't get measurement in interval"
        self.lock.release()

    def add_cam_measurement(self, cam_id, msg):
        #print "Adding [%s]" % cam_id
        if len(msg.image_points) > 0:
            self.lock.acquire()
            self.cache.add_cam_measurement(cam_id, msg)
            self.lock.release()

if __name__ == '__main__':
    rospy.init_node('capture_exec')
    executive = CameraCaptureExecutive(['camera_a', 'camera_b'])
    rospy.spin()
