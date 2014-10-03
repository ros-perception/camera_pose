#!/usr/bin/python

import tf2_ros
import rospy
import PyKDL
import threading
from tf_conversions import posemath
from geometry_msgs.msg import TransformStamped, Pose
from camera_pose_calibration.msg import CameraCalibration


class CameraPublisher:
    def __init__(self, pose, child_frame_id):
        self.lock = threading.Lock()
        self.pub = tf2_ros.TransformBroadcaster()
        self.set_pose(pose, child_frame_id)

    def set_pose(self, pose, child_frame_id):
        with self.lock:
            self.transform = TransformStamped()
            self.transform.header.frame_id = 'world'
            self.transform.child_frame_id = child_frame_id
            self.transform.transform.translation.x = pose.position.x
            self.transform.transform.translation.y = pose.position.y
            self.transform.transform.translation.z = pose.position.z
            self.transform.transform.rotation.x = pose.orientation.x
            self.transform.transform.rotation.y = pose.orientation.y
            self.transform.transform.rotation.z = pose.orientation.z
            self.transform.transform.rotation.w = pose.orientation.w

    def publish(self):
        with self.lock:
            self.transform.header.stamp = rospy.Time.now() + rospy.Duration(0.5)
            self.pub.sendTransform(self.transform)



class CalibrationPublishManager:
    def __init__(self):
        self.lock = threading.Lock()
        self.publish_list = {}
        self.sub = rospy.Subscriber('camera_calibration', CameraCalibration, self.cal_cb)

    def cal_cb(self, msg):
        with self.lock:
            self.publish_list = {}
            for pose, camera in zip(msg.camera_pose, msg.camera_id):
                self.publish_list[camera] = CameraPublisher(pose, camera)


    def publish(self):
        with self.lock:
            for name, pub in self.publish_list.iteritems():
                pub.publish()



def main():
    rospy.init_node('calibration_tf_publisher')
    r = rospy.Rate(5)
    c = CalibrationPublishManager()
    while not rospy.is_shutdown():
        c.publish()
        try:
            r.sleep()
        except:
            print "Shutting down"


if __name__ == '__main__':
    main()
