#! /usr/bin/env python

import roslib
roslib.load_manifest('camera_pose_calibration')

import itertools
import collections
import rospy
import threading

from tf_conversions import posemath
from std_msgs.msg import Empty
from camera_pose_calibration.msg import CalibrationEstimate
from camera_pose_calibration.msg import CameraPose
from camera_pose_calibration.msg import RobotMeasurement, CameraCalibration
from camera_pose_calibration import init_optimization_prior
from camera_pose_calibration import estimate
from camera_pose_calibration import camera_info_converter

class Estimator:
    def __init__(self):
        self.lock = threading.Lock()
        self.reset()
        self.pub = rospy.Publisher('camera_calibration', CameraCalibration)
        self.sub_reset = rospy.Subscriber('reset', Empty, self.reset_cb)
        self.sub_meas  = rospy.Subscriber('robot_measurement', RobotMeasurement, self.meas_cb)

    def reset_cb(self, msg):
        self.reset()

    def reset(self):
        with self.lock:
            self.state = None
            self.meas = []
            rospy.loginfo('Reset calibration state')


    def meas_cb(self, msg):
        with self.lock:
            # check if cam_info is valid
            for camera in msg.M_cam:
                P = camera.cam_info.P
                all_zero = True
                for i in range(12):
                    if P[i] != 0:
                        all_zero = False
                if all_zero:
                    rospy.logfatal("Camera info of %s is all zero. You should calibrate your camera intrinsics first "%camera.camera_id)
                    exit(-1)

            # Modify all the camera info messages to make sure that ROI and binning are removed, and P is updated accordingly
            # Update is done in place
            #for camera in msg.M_cam:
            #    camera.cam_info = camera_info_converter.unbin(camera.cam_info)

            # add measurements to list
            self.meas.append(msg)
            print "MEAS", len(self.meas)
            for m in self.meas:
                print " - stamp: %f"%m.header.stamp.to_sec()

            # initialize state if needed
            if True: #not self.state:
                self.state = CalibrationEstimate()
                camera_poses, checkerboard_poses = init_optimization_prior.find_initial_poses(self.meas)
                self.state.targets = [ posemath.toMsg(checkerboard_poses[i]) for i in range(len(checkerboard_poses)) ]
                self.state.cameras = [ CameraPose(camera_id, posemath.toMsg(camera_pose)) for camera_id, camera_pose in camera_poses.iteritems()]

            # run optimization
            self.state = estimate.enhance(self.meas, self.state)

            # publish calibration state
            res = CameraCalibration()
            res.camera_pose = [camera.pose for camera in self.state.cameras]
            res.camera_id = [camera.camera_id for camera in self.state.cameras]
            self.pub.publish(res)


def main():
    rospy.init_node('online_calibration')
    e = Estimator()

    rospy.spin()

if __name__ == '__main__':
    main()



