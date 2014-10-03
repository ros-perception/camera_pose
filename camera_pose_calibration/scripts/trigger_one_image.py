#!/usr/bin/python

import rospy
from camera_pose_calibration.srv import TriggerOne, TriggerOneResponse
from sensor_msgs.msg import Image

class TriggerOneMessageServer:
    def __init__(self):
        self.srv = rospy.Service('trigger_one', TriggerOne, self.srv_cb)
        self.pub = rospy.Publisher('output', Image)


    def srv_cb(self, req):
        rospy.logdebug('In service call, waiting for message')
        msg = rospy.wait_for_message('input', Image)
        rospy.logdebug('Received message')
        self.pub.publish(msg)
        return TriggerOneResponse()


def main():
    rospy.init_node('trigger_one_image')
    t = TriggerOneMessageServer()
    rospy.spin()


if __name__ == '__main__':
    main()
