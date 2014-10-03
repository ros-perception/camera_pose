#!/usr/bin/python

import copy
import rospy
import threading
from sensor_msgs.msg import CameraInfo


def unbin(msg_in):
    '''
    Modify projection matrix such that it corresponds to the binned image
    '''
    msg_out = copy.deepcopy(msg_in)
    b_x = msg_in.binning_x
    b_y = msg_in.binning_y
    off_x = msg_in.roi.x_offset
    off_y = msg_in.roi.y_offset

    P = list(msg_in.P)

    if b_x == 0:
        b_x = 1
    if b_y == 0:
        b_y = 1

    P[0] /= b_x
    P[1] /= b_x
    P[2] = (P[2] - off_x)/b_x
    P[3] /= b_x

    P[4] /= b_y
    P[5] /= b_y
    P[6] = (P[6] - off_y)/b_y
    P[7] /= b_y

    msg_out.P = P
    msg_out.binning_x = 1
    msg_out.binning_y = 1

    msg_out.height /= b_x
    msg_out.width /= b_y

    msg_out.roi.x_offset = 0
    msg_out.roi.y_offset = 0
    msg_out.roi.height = msg_out.height
    msg_out.roi.width = msg_out.width

    msg_out.D = msg_in.D
    msg_out.R = msg_in.R
    msg_out.K = (P[0], P[1], P[2],
                 P[4], P[5], P[6],
                 P[8], P[9], P[10])
    return msg_out;


class CameraInfoConverter:
    def __init__(self):
        self.lock = threading.Lock()
        self.pub_interval = rospy.Duration(rospy.get_param('~publish_interval', 0.0))
        self.last_pub = rospy.Time()
        self.pub = rospy.Publisher('camera_info_out', CameraInfo)
        self.sub = rospy.Subscriber('camera_info_in', CameraInfo, self.cam_info_cb)

    def cam_info_cb(self, msg):
        with self.lock:
            time_now = rospy.Time.now()
            if self.last_pub + self.pub_interval < time_now:
                self.pub.publish(unbin(msg))
                self.last_pub = time_now
