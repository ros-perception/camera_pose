#!/usr/bin/python

import roslib; roslib.load_manifest('megacal_estimation')
import tf2_ros
import rospy
import threading
import PyKDL
from tf_conversions import posemath
from geometry_msgs.msg import TransformStamped
from megacal_estimation.msg import CameraCalibration

name_mapping = {'openni_rgb_optical_frame':'openni_camera'}

def get_parent(name):
    res = name
    for child, parent in name_mapping.iteritems():
        res = res.replace(child, parent)
    return res


def fromTrMsg(t):
    return PyKDL.Frame(PyKDL.Rotation.Quaternion(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w),
                       PyKDL.Vector(t.translation.x, t.translation.y, t.translation.z))

class CameraPublisher(threading.Thread):
    def __init__(self, name, pose):
        threading.Thread.__init__(self)
        self.lock = threading.Lock()
        self.name = name+'/openni_rgb_optical_frame'  # what is name used by calibration?
        self.pose = pose
        self.pub = tf2_ros.TransformBroadcaster()
        self.start()

    def set_pose(self, pose):
        with self.lock:
            self.pose = pose

    def run(self):
        rospy.loginfo("Connecting to tf2 server")
        tf = tf2_ros.BufferClient('tf2_buffer_server')
        tf.wait_for_server()
        rospy.loginfo('Looking up tf offset from frame %s to frame %s'%(self.name, get_parent(self.name)))
        offset = fromTrMsg(tf.lookup_transform(get_parent(self.name), self.name, rospy.Time(),
                                               rospy.Duration(100)).transform)

        rospy.loginfo('Start publishing tf for frame %s'%self.name)
        while not rospy.is_shutdown():
            with self.lock:
                res = posemath.toMsg(posemath.fromMsg(self.pose) * offset.Inverse())
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
