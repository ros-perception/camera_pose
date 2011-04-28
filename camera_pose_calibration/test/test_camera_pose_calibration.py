#!/usr/bin/env python

PKG = 'camera_pose_calibration'
import roslib; roslib.load_manifest(PKG)
import rospy
#import tf2_ros
#import tf2_kdl
import tf
import PyKDL
import unittest
import rostest


class TestCameraPose(unittest.TestCase):
    def test_one(self):
        #tf_buffer = tf2_ros.Buffer()
        #tf_listener = tf2_ros.TransformListener(tf_buffer)
        tf_listener = tf.TransformListener()
        fr1 = 'camera_a/openni_depth_optical_frame'
        fr2 = 'camera_c/prosilica_optical_frame'

        #tr_check = PyKDL.Frame(PyKDL.Rotation.Quaternion(0.022, 0.007, -0.006, 1.000), PyKDL.Vector(-0.011, 0.027, 0.048))
        #tr_identity = tf2_ros.Stamped(PyKDL.Frame.Identity(), rospy.Time(0), fr2)
        #tr_res = tf_buffer.transform(tr_identity, fr1, rospy.Duration(60))
        #diff = PyKDL.diff(tr_check*tr_res.Inverse(), PyKDL.Frame.Identity())
        #self.assertAlmostEqual(diff.rot.Norm(), 0.0, 1)
        #self.assertAlmostEqual(diff.vel.Norm(), 0.0, 1)

        tf_listener.waitForTransform(fr1, fr2, rospy.Time(0), rospy.Duration(60))
        ((x, y, z), (rx, ry, rz, rw)) = tf_listener.lookupTransform(fr1, fr2, rospy.Time(0))
        self.assertAlmostEqual(x, -0.011, 1)
        self.assertAlmostEqual(y, 0.027, 1)
        self.assertAlmostEqual(z, 0.048, 1)
        self.assertAlmostEqual(rx, 0.022, 1)
        self.assertAlmostEqual(ry, 0.007, 1)
        self.assertAlmostEqual(rz, -0.006, 1)
        self.assertAlmostEqual(rw, 1.00, 1)
        


if __name__ == "__main__":
    rospy.init_node('test_camera_pose_calibration')
    rostest.rosrun(PKG, 'test_camera_pose_calibration', TestCameraPose)
