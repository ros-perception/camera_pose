#!/usr/bin/python

import tf2_ros
import rospy
import PyKDL
from tf_conversions import posemath
from geometry_msgs.msg import TransformStamped


class StaticPublisher:
    def __init__(self):
        self.pub = tf2_ros.TransformBroadcaster()
        pose = rospy.get_param('~transform')
        self.pose_msg = TransformStamped()
        self.pose_msg.header.frame_id = pose['parent_id']
        self.pose_msg.child_frame_id = pose['child_id']
        self.pose_msg.transform.translation.x = pose['translation'][0]
        self.pose_msg.transform.translation.y = pose['translation'][1]
        self.pose_msg.transform.translation.z = pose['translation'][2]
        q = self.pose_msg.transform.rotation

        if 'rotation' in pose and 'quaternion' in pose:
            rospy.logfatal("'quaterion' and 'rotation' specified. Please remove one")
            raise

        if 'rotation' in pose:
            (q.x, q.y, q.z, q.w) = PyKDL.Rotation(*pose['rotation']).GetQuaternion()
        elif 'quaternion' in pose:
            r = pose['quaternion']
            q.x = r['x']
            q.y = r['y']
            q.z = r['z']
            q.w = r['w']
        else:
            rospy.logfatal("No rotation specified. Use 'rotation' or 'quaternion'.")
            raise
             


    def publish(self):
        self.pose_msg.header.stamp = rospy.Time.now() + rospy.Duration(0.5)
        self.pub.sendTransform(self.pose_msg)



def main():
    rospy.init_node('static_tf_publisher')
    s = StaticPublisher()
    r = rospy.Rate(5)
    while not rospy.is_shutdown():
        s.publish()
        r.sleep()


if __name__ == '__main__':
    main()
