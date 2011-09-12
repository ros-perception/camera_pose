#!/usr/bin/env python

PKG = 'camera_pose_calibration'
import roslib; roslib.load_manifest(PKG)
import rospy
import tf
import PyKDL
import unittest
import rostest


class TestCameraPose(unittest.TestCase):
    def test_one(self):
        tf_listener = tf.TransformListener()
        fr1 = 'camera_a/openni_depth_optical_frame'
        fr2 = 'camera_c/prosilica_optical_frame'

        tf_listener.waitForTransform(fr1, fr2, rospy.Time(0), rospy.Duration(400))
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
    rospy.sleep(0.1)
    rostest.rosrun(PKG, 'test_camera_pose_calibration', TestCameraPose)
