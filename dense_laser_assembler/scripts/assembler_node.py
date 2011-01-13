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
#


import roslib; roslib.load_manifest('dense_laser_assembler')
import sys
import rospy
import threading
from time import sleep
from sensor_msgs.msg import LaserScan
from pr2_msgs import LaserScannerSignal
from mechanism_msgs.msg import MechanismState
from std_msgs.msg import *
from dense_laser_assembler.msg import *
from pr2_mechanism_controllers.msg import *
from dense_laser_assembler.dense_laser_cache import DenseLaserCache
from sensor_msgs.msg import *

# Globals
laser_cache = [ ]
first_signal = True
prev_signal_time = [ ]
image_pub = [ ]
test_pub = [ ]


def scan_callback(msg) :
    req_lock.acquire()
    laser_cache.add_scan(msg)
    laser_cache.process_scans()
    laser_cache.process_interval_reqs()
    req_lock.release()

def mech_callback(msg) :
    req_lock.acquire()
    laser_cache.add_mech_state(msg)
    laser_cache.process_scans()
    laser_cache.process_interval_reqs()
    req_lock.release()

def signal_callback(msg) :
    req_lock.acquire()
    global first_signal
    global prev_signal
    laser_cache.process_scans()
    if first_signal :
        first_signal = False
    else :
        laser_cache.add_interval_req(prev_signal.header.stamp, msg.header.stamp)
        laser_cache.process_interval_reqs()
    prev_signal = msg
    print 'Got signal message!'
    print '  Signal=%i' % msg.signal
    req_lock.release()

def interval_req_callback(scans) :

    # Check data consistency
    if (len(scans) == 0) :
        print 'Processed interval with no scans. Not publishing'
        return

    rows = len(scans)
    cols = len(scans[0].scan.ranges)
    if any( [len(x.scan.ranges) != cols for x in scans] ) or any( [len(x.scan.intensities) != cols for x in scans]) :
        print "# Readings aren't consistent across interval"
        return


    print 'About to process cloud with %u scans' % len(scans)

    # Sort by tilting joint angle
    sorted_scans = sorted(scans, lambda x,y:cmp(x.pos[0],y.pos[0]))


    msg_header = rospy.Header()
    msg_header.stamp = scans[-1].scan.header.stamp
    msg_header.frame_id = scans[-1].scan.header.frame_id

    layout = MultiArrayLayout([ MultiArrayDimension('rows', rows, rows*cols),
                                MultiArrayDimension('cols', cols, cols),
                                MultiArrayDimension('elem', 1, 1) ], 0 )
    # Build the messages in a reasonable ROS format
    intensity_msg = Float32MultiArrayStamped()
    range_msg = Float32MultiArrayStamped()
    info_msg  = scans[-1].scan
    joint_msg = Float32MultiArrayStamped()

    intensity_msg.header = msg_header
    range_msg.header     = msg_header
    joint_msg.header     = msg_header

    intensity_msg.data.layout = layout
    range_msg.data.layout     = layout
    joint_msg.data.layout = MultiArrayLayout([ MultiArrayDimension('rows', rows, rows*2),
                                               MultiArrayDimension('cols', 2, 2),
                                               MultiArrayDimension('elem', 1, 1) ], 0 )
    #joint_msg.data.layout = layout

    # Clear out data from the info message. (Keep everything except intensity and range data)
    #info_msg.ranges = [ ]
    #info_msg.intensities = [ ]

    # Populate intensities & ranges
    intensity_msg.data.data = []
    range_msg.data.data = []
    for x in sorted_scans :
        intensity_msg.data.data.extend(x.scan.intensities)
        range_msg.data.data.extend(x.scan.ranges)
        joint_msg.data.data.extend(x.pos)

    intensity_pub.publish(intensity_msg)
    range_pub.publish(range_msg)
    info_pub.publish(info_msg)
    joint_pos_pub.publish(joint_msg)

    #image = sensor_msgs.msg.Image()

    #image.header = msg_header
    #image.label  = 'intensity'
    #image.encoding = 'mono'
    #image.depth = 'uint8'
    #max_val = max(joint_msg.data.data)
    #min_val = min(joint_msg.data.data)
    #image.uint8_data.data   = "".join([chr(int(255*x/max_val)) for x in intensity_msg.data.data])
    #image.uint8_data.data   = "".join([chr(int(255*(x-min_val)/(max_val-min_val))) for x in joint_msg.data.data])
    #image.uint8_data.data = ''
    #image.uint8_data.layout = intensity_msg.data.layout
    #image_pub.publish(image)


    print '  Processed cloud with %u rays' % len(intensity_msg.data.data)

if __name__ == '__main__':

    req_lock = threading.Lock()

    print 'Initializing DenseLaserCache'
    laser_cache = DenseLaserCache(interval_req_callback, 200, 1000, 100) ;
    print 'Done initializing'

    rospy.init_node('dense_assembler', anonymous=False)
    signal_sub = rospy.Subscriber("laser_tilt_controller/laser_scanner_signal",
                                  LaserScannerSignal, signal_callback)

    intensity_pub = rospy.Publisher("dense_tilt_scan/intensity", Float32MultiArrayStamped)
    range_pub     = rospy.Publisher("dense_tilt_scan/range",     Float32MultiArrayStamped)
    info_pub      = rospy.Publisher("dense_tilt_scan/scan_info", LaserScan)
    joint_pos_pub = rospy.Publisher("dense_tilt_scan/joint_info",Float32MultiArrayStamped)
    #image_pub = rospy.Publisher("test_image", Image)

    scan_sub = rospy.Subscriber("tilt_scan", LaserScan, scan_callback)

    mech_sub = rospy.Subscriber("mechanism_state", MechanismState, mech_callback)

    rospy.spin()
