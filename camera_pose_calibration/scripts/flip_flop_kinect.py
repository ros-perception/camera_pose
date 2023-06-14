#!/usr/bin/python

import rospy
from camera_pose_calibration.srv import TriggerOne, TriggerOneRequest



def main():
    rospy.init_node('flip_flop_kinect')
    rospy.wait_for_service('trigger_one_1')
    rospy.wait_for_service('trigger_one_2')
    srv1 = rospy.ServiceProxy('trigger_one_1', TriggerOne)
    srv2 = rospy.ServiceProxy('trigger_one_2', TriggerOne)

    while not rospy.is_shutdown():
        rospy.loginfo("Triggering kinect rgb and kinect ir")
        srv1.call(TriggerOneRequest())
        srv2.call(TriggerOneRequest())


if __name__ == '__main__':
    main()
