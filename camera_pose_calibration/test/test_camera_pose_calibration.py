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

        tr_check = PyKDL.Frame(PyKDL.Rotation.Quaternion(0.022, 0.007, -0.006, 1.000), PyKDL.Vector(-0.011, 0.027, 0.048))
        tr_identity = tf2_ros.Stamped(PyKDL.Frame.Identity(), rospy.Time(0), 'camera_c/prosilica_optical_frame')
        rospy.sleep(15)
        tr_res = tf_buffer.transform(tr_identity, 'camera_a/openni_depth_optical_frame', rospy.Duration(60))
        diff = PyKDL.diff(tr_check*tr_res.Inverse(), PyKDL.Frame.Identity())

        print "tr_check" , tr_check.p, tr_check.M.GetQuaternion()
        print "tr_check_inv" , tr_check.Inverse().p, tr_check.Inverse().M.GetQuaternion()
        print "tr_res" , tr_res.p, tr_res.M.GetQuaternion()
        print "tr_res_inv" , tr_res.Inverse().p, tr_res.Inverse().M.GetQuaternion()
        print "Diff " , diff
        self.assertAlmostEqual(diff.rot.Norm(), 0.0, 1)
        self.assertAlmostEqual(diff.vel.Norm(), 0.0, 1)
        


if __name__ == "__main__":
    rospy.init_node('test_camera_pose_calibration')
    rostest.rosrun(PKG, 'test_camera_pose_calibration', TestCameraPose)
