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

from pr2_calibration_estimation.sensors import tilting_laser_sensor, chain_sensor, camera_chain_sensor

class MultiSensor:
    '''
    Provides helper methods for dealing with all the sensor measurements
    generated from a single RobotMeasurement/CbPose pair
    '''
    def __init__(self, sensor_configs):
        self._sensor_configs = sensor_configs

    def sensors_from_message(self, msg):
        sensors = []

        sensor_type = 'tilting_lasers'
        if sensor_type in self._sensor_configs.keys():
            cur_bundler = tilting_laser_sensor.TiltingLaserBundler( self._sensor_configs[sensor_type] )
            cur_sensors = cur_bundler.build_blocks(msg)
            sensors.extend(cur_sensors)
        else:
            print "[%s] section doesn't exist. Skipping"

        sensor_type = 'chains'
        if sensor_type in self._sensor_configs.keys():
            cur_bundler = chain_sensor.ChainBundler( self._sensor_configs[sensor_type] )
            cur_sensors = cur_bundler.build_blocks(msg)
            sensors.extend(cur_sensors)
        else:
            print "[%s] section doesn't exist. Skipping"

        sensor_type = 'camera_chains'
        if sensor_type in self._sensor_configs.keys():
            cur_bundler = camera_chain_sensor.CameraChainBundler( self._sensor_configs[sensor_type] )
            cur_sensors = cur_bundler.build_blocks(msg)
            sensors.extend(cur_sensors)
        else:
            print "[%s] section doesn't exist. Skipping"

        # Store the sensor list internally
        self.sensors = sensors
        self.checkerboard = msg.target_id


