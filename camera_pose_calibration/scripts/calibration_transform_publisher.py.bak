#!/usr/bin/python

import tf
import rospy
import threading
from camera_pose_calibration.msg import CameraCalibration

class Transform:
    def __init__(self):
	self.parent_frame_id = 'world'
        self.child_frame_id = 'link'
        self.translation = (0,0,0)
        self.rotation = (0,0,0,0)
        self.stamp = rospy.Time.now()

class CameraPublisher:
    def __init__(self, pose, child_frame_id):
        self.lock = threading.Lock()
        self.pub = tf.TransformBroadcaster()
        self.set_pose(pose, child_frame_id)

    def set_pose(self, pose, child_frame_id):
        with self.lock:
            self.transform = Transform()
            self.transform.parent_frame_id = 'world'
            self.transform.child_frame_id = child_frame_id
            self.transform.translation = (pose.position.x, 
                                          pose.position.y, 
                                          pose.position.z)
            self.transform.rotation = (pose.orientation.x, 
                                       pose.orientation.y, 
                                       pose.orientation.z, 
                                       pose.orientation.w)

    def publish(self):
        with self.lock:
            self.transform.stamp = rospy.Time.now() + rospy.Duration(0.5)
            self.pub.sendTransform(self.transform.translation,
				   self.transform.rotation,
				   self.transform.stamp,
				   self.transform.child_frame_id,
                                   self.transform.parent_frame_id)



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

