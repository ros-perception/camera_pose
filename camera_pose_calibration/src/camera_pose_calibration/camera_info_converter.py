#!/usr/bin/python

import roslib; roslib.load_manifest('camera_pose_calibration')
import rospy
from sensor_msgs.msg import CameraInfo





class CameraInfoConverter:
    def __init__(self):
        self.pub = rospy.Publisher('camera_info_out', CameraInfo)
        self.sub = rospy.Subscriber('camera_info_in', CameraInfo, self.cam_info_cb)


    def cam_info_cb(self, msg):
        b_x = msg.binning_x
        b_y = msg.binning_y
        off_x = msg.roi.x_offset
        off_y = msg.roi.y_offset

        P = list(msg.P)

        P[0] /= b_x
        P[1] /= b_x
        P[2] = (P[2] - off_x)/b_x
        P[3] /= b_x

        P[4] /= b_y
        P[5] /= b_y
        P[6] = (P[6] - off_y)/b_y
        P[7] /= b_y

        msg.P = P
        msg.binning_x = 1
        msg.binning_y = 1

        msg.height /= b_x
        msg.width /= b_y

        msg.roi.x_offset = 0
        msg.roi.y_offset = 0
        msg.roi.height = msg.height
        msg.roi.width = msg.width

        msg.D = []
        msg.K = (P[0], P[1], P[2],
                 P[4], P[5], P[6],
                 P[8], P[9], P[10])

        self.pub.publish(msg)




def main():
    rospy.init_node('camera_info_converter')
    camera_converter = CameraInfoConverter()
    rospy.spin()



if __name__ == '__main__':
    main()
