#!/usr/bin/python

import roslib; roslib.load_manifest('megacal_estimation')
import tf2_ros
import rospy
import threading
from tf_conversions import posemath
from geometry_msgs.msg import TransformStamped
from calibration_msgs.msg import CameraCalibration

name_mapping = {'openni_rgb_optical_frame':'openni_camera'}

def get_parent(name):
    for child, parent in name_mapping.iteritems():
        name = name.replace(child, parent)
    return name


class CameraPublisher(threading.Thread):
    def __init__(self, name, pose):
        threading.Thread.__init__(self)
        self.lock = threading.Lock()
        self.name = name
        self.pose = pose
        self.pub = tf2_ros.TransformBroadcaster()
        self.start()

    def set_pose(self, pose):
        with self.lock:
            self.pose = pose

    def run(self):
        rospy.loginfo('Looking up tf offset for frame %s'%self.name)
        tf = tf2_ros.BufferClient('tf2_server')
        tf.wait_for_server()
        offset = posemath.fromMsg(tf.lookup_transform(get_parent(self.name), self.name, rospy.Time(),
                                                      rospy.Duration(100)).pose)

        while not rospy.is_shutdown():
            rospy.loginfo('Start publishing tf for frame %s'%self.name)
            with self.lock:
                res = posemath.toMsg(posemath.fromMsg(self.pose) * offset)
            transform = TransformStamped()
            transform.header.frame_id = 'world'
            transform.child_frame_id = get_parent(self.name)
            transform.header.stamp = rospy.Time.now() + rospy.Duration(0.5)
            transform.transform.translation.x = res.position.x
            transform.transform.translation.y = res.position.y
            transform.transform.translation.z = res.position.z
            transform.transform.rotation.x = res.orientation.x
            transform.transform.rotation.y = res.orientation.y
            transform.transform.rotation.z = res.orientation.z
            transform.transform.rotation.w = res.orientation.w
            self.pub.sendTransform(transform)
            rospy.sleep(0.1)


class MultiCameraPublisher:
    def __init__(self):
        self.publish_list = {}
        self.sub = rospy.Subscriber('camera_calibration', CameraCalibration, self.cal_cb)
        

    def cal_cb(self, msg):
        for pose, camera in zip(msg.camera_pose, msg.camera_id):
            # add new entry
            if not camera in self.publish_list:
                self.publish_list[camera] = CameraPublisher(camera, pose)
            # modify existing entry
            else:
                self.publish_list[camera].set_pose(pose)



def main():
    rospy.init_node('cal_tf_publisher')
    p = MultiCameraPublisher()
    rospy.spin()


if __name__ == '__main__':
    main()
