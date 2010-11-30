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
from time import sleep
from laser_scan.msg import *
from std_msgs.msg import *
from dense_laser_assembler.msg import *
from pr2_mechanism_controllers.msg import *
from sensor_msgs.msg import Image


# Takes the dense laser scan data and converts it into images to can be viewed in
# image_view. Generally used as a debugging tool.


def rescale(x, max_val) :
    lower_lim = max_val*0.0
    upper_lim = max_val*1.0
    #lower_lim = 2000
    #upper_lim = 3000
    a = x - lower_lim
    if a <= 0 :
        return chr(0)
    elif a > upper_lim -lower_lim :
        return chr(255)
    
    return chr(int((2**8-1)*a/(upper_lim-lower_lim)))

def intensity_callback(msg) :
    image = sensor_msgs.msg.Image()
    image.header = msg.header
    image.label  = 'intensity'
    image.encoding = 'mono'
    image.depth = 'uint8'
    max_val = max(msg.data.data)
    #image.uint8_data.data   = "".join([chr(int((2**8-1)*x/max_val)) for x in msg.data.data])
    image.uint8_data.data   = "".join([rescale(x,max_val) for x in msg.data.data])
    image.uint8_data.layout = msg.data.layout
    intensity_pub.publish(image)

def range_callback(msg) :
    image = sensor_msgs.msg.Image()
    image.header = msg.header
    image.label  = 'range'
    image.encoding = 'mono'
    image.depth = 'uint8'
    max_val = max(msg.data.data)
    image.uint8_data.data   = "".join([chr(int((2**8-1)*x/max_val)) for x in msg.data.data])
    image.uint8_data.layout = msg.data.layout
    range_pub.publish(image)

if __name__ == '__main__':

    rospy.init_node('laser_imager', anonymous=False)

    intensity_pub = rospy.Publisher("dense_tilt_scan/intensity_image", Image)
    range_pub     = rospy.Publisher("dense_tilt_scan/range_image",     Image)
    
    intensity_sub = rospy.Subscriber("dense_tilt_scan/intensity", Float32MultiArrayStamped, intensity_callback)
    range_sub     = rospy.Subscriber("dense_tilt_scan/range",     Float32MultiArrayStamped, range_callback)

    rospy.spin()
