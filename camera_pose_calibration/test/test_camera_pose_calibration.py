#!/usr/bin/env python

PKG = 'camera_pose_calibration'
import roslib; roslib.load_manifest(PKG)
import rospy
import tf2_ros
import tf2_kdl
import PyKDL
import unittest
import rostest


class TestCameraPose(unittest.TestCase):
    def test_one(self):
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)

        eps_rot = 1e-1
        eps_vel = 1e-2

        tr_check = PyKDL.Frame(PyKDL.Rotation.Quaternion(0.022, 0.007, -0.006, 1.000), PyKDL.Vector(-0.011, 0.027, 0.048))
        tr_identity = tf2_ros.Stamped(PyKDL.Frame.Identity(), rospy.Time(0), 'camera_a/openni_depth_optical_frame')
        tr_res = tf_buffer.transform(tr_identity, 'camera_c/prosilica_optical_frame', rospy.Duration(60))
        diff = PyKDL.diff(tr_check, tr_res)
        self.assertAlmostEqual(diff.rot.Norm(), 0.0, delta=eps_rot)
        self.assertAlmostEqual(diff.vel.Norm(), 0.0, delta=eps_vel)
        


if __name__ == "__main__":
    rospy.init_node('test_camera_pose_calibration')
    rostest.rosrun(PKG, 'test_camera_pose_calibration', TestCameraPose)
