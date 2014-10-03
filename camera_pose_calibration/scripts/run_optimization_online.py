#! /usr/bin/env python

import itertools
import collections
import rospy
import threading
import tf

import math

from tf_conversions import posemath
from std_msgs.msg import Empty
from camera_pose_calibration.msg import CalibrationEstimate
from camera_pose_calibration.msg import CameraPose
from camera_pose_calibration.msg import RobotMeasurement, CameraCalibration
from camera_pose_calibration import init_optimization_prior
from camera_pose_calibration import estimate
from camera_pose_calibration import camera_info_converter
from std_msgs.msg import Time
from sensor_msgs.msg import CameraInfo

class Estimator:
    def __init__(self):
        self.lock = threading.Lock()
        self.meas = []  ##
        self.reset()


        self.prev_tf_transforms = {}
        self.tf_check_pairs = []
        self.tf_listener = tf.TransformListener() 

        self.measurement_count = 0
        self.timestamps =[]

        self.pub = rospy.Publisher('camera_calibration', CameraCalibration)
        self.sub_reset = rospy.Subscriber('reset', Empty, self.reset_cb)
        self.sub_meas  = rospy.Subscriber('robot_measurement', RobotMeasurement, self.meas_cb)

        self.prev_tf_pair=()
        self.prev_3_frames=()
        self.reset_flag = False
        self.frames_changed = False


    def reset_cb(self, msg):
        self.reset()

    def reset(self):
        with self.lock:
            self.state = None
            count = len(self.meas)
            self.meas = []
            self.timestamps = []
            rospy.loginfo('Reset calibration state')
            print "\033[33;1m%ld\033[0m previous measurements in the cache are deleted.\n\n" % count

    def getAngle(self, quaternion): 
        if (math.fabs(quaternion[3])>1):
            print "quaternion[3] out of range"  # send out a notice only
        s = 2.* math.acos(quaternion[3])
        return s
                   
    def getAxis(self, quaternion):
        s_squared = 1. - math.pow(quaternion[3], 2.)
        if s_squared < 10. * 1e-5 : #FLT_EPSILON 1e-5
            return (1.0, 0.0, 0.0) #Arbitrary
        s = math.sqrt(s_squared)
        return (quaternion[0]/s, quaternion[1]/s, quaternion[2]/s)

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

            if rospy.has_param('optimizer_conditional_resetter_enable'):
                self.reset_flag = rospy.get_param('optimizer_conditional_resetter_enable')
            else:
                self.reset_flag = False

            if self.reset_flag == True :                  
                urdf_cam_ns = rospy.get_param("urdf_cam_ns")
                new_cam_ns = rospy.get_param("new_cam_ns")
                u_info = rospy.wait_for_message(urdf_cam_ns+'/camera_info', CameraInfo)
                n_info = rospy.wait_for_message(new_cam_ns+'/camera_info', CameraInfo)
                urdf_cam_frame = u_info.header.frame_id
                new_cam_frame = n_info.header.frame_id
                mounting_frame = rospy.get_param("mounting_frame")
            
                # if any of the frames has changed...
                if self.prev_3_frames != () :
                    if self.prev_3_frames != (urdf_cam_frame, new_cam_frame, mounting_frame):
                        self.frames_changed = True       

                if self.frames_changed :
                    self.state = None  
                    self.meas = []  # clear cache
                    self.timestamps = []
                    self.measurement_count = 0
                    self.prev_tf_transforms = {}
                    rospy.loginfo('Reset calibration state')
                    print "\033[43;1mTarget frames have changed. Clean up everything and start over! \033[0m\n\n"
                    self.frames_changed = False

           
                if len(self.tf_check_pairs) > 0 :
                    self.prev_tf_pair = self.tf_check_pairs[0]
                self.tf_check_pairs = []
                self.tf_check_pairs.append((mounting_frame, urdf_cam_frame)) # only keep one pair in the list at a time

                self.prev_3_frames = (urdf_cam_frame, new_cam_frame, mounting_frame)
                self.prev_tf_pair = (mounting_frame, urdf_cam_frame)

            TheMoment = msg.header.stamp

            # check for tf transform changes
            for (frame1, frame2) in self.tf_check_pairs:
                transform = None
                print "\nLooking up transformation \033[34;1mfrom\033[0m %s \033[34;1mto\033[0m %s ..." % (frame1, frame2)
                while not rospy.is_shutdown():
                    try:                
                        self.tf_listener.waitForTransform(frame1, frame2, TheMoment, rospy.Duration(10))
                        transform = self.tf_listener.lookupTransform(frame1, frame2, TheMoment)  #((),())
                        print "found\n"
                        break
                    except (tf.LookupException, tf.ConnectivityException):
                        print "transform lookup failed, retrying..."

                if self.measurement_count > 0:
                    prev_transform = self.prev_tf_transforms[(frame1, frame2)]
                    dx = transform[0][0] - prev_transform[0][0]
                    dy = transform[0][1] - prev_transform[0][1]
                    dz = transform[0][2] - prev_transform[0][2]
                    dAngle = self.getAngle(transform[1]) - self.getAngle(prev_transform[1])
                    a=self.getAxis(transform[1])
                    b=self.getAxis(prev_transform[1])
                    dAxis = (a[0]*b[0] + a[1]*b[1] + a[2]*b[2])/1. # a dot b
                
                    #print "\n"
                    #print "\033[34;1m-- Debug Info --\033[0m"
                    print "checking for pose change..."
                    print "measurement_count = %d" % self.measurement_count
                    print "    dx = %10f | < 0.0002 m" % dx
                    print "    dy = %10f | < 0.0002 m" % dy
                    print "    dz = %10f | < 0.0002 m" % dz
                    print "dAngle = %10f | < 0.00174 rad" % dAngle
                    print " dAxis = %10f | > 0.99999\n" % dAxis

                    if ( (math.fabs(dx) > 0.0002) or (math.fabs(dy) > 0.0002) or (math.fabs(dz) > 0.0002) or
                            (math.fabs(dAngle)>0.00174) or (math.fabs(dAxis) < 0.99999) ) :  # if transform != previous_transform:
                        print "\033[44;1mDetect pose change: %s has changed pose relative to %s since last time!\033[0m\n\n" % (frame1, frame2)
                                ###self.reset()  # no, you cannot call reset() here.
                        
                        self.state = None  
                        count = len(self.meas)
                        self.meas = []  # clear cache
                        self.timestamps = []
                        rospy.loginfo('Reset calibration state')
                        print "\033[33;1m%ld\033[0m previous measurements in the cache are deleted.\n\n" % count
                    self.measurement_count = 0

                self.prev_tf_transforms[(frame1, frame2)] = transform

            # add timestamp to list
            self.timestamps.append(msg.header.stamp)

            # add measurements to list
            self.meas.append(msg)
            print "MEAS", len(self.meas)
            for m in self.meas:
                print " - stamp: %f"%m.header.stamp.to_sec()
            
            print "\n"
            
            self.measurement_count += 1

            print "\nNo. of measurements fed to optimizer: %d\n\n" %  self.measurement_count

            ## self.last_stamp_pub.publish(self.meas[-1].header.stamp)

            # initialize state if needed
            if True: #not self.state:
                self.state = CalibrationEstimate()
                camera_poses, checkerboard_poses = init_optimization_prior.find_initial_poses(self.meas)
                self.state.targets = [ posemath.toMsg(checkerboard_poses[i]) for i in range(len(checkerboard_poses)) ]
                self.state.cameras = [ CameraPose(camera_id, posemath.toMsg(camera_pose)) for camera_id, camera_pose in camera_poses.iteritems()]

            print "Proceed to optimization...\n"

            # run optimization
            self.state = estimate.enhance(self.meas, self.state)

            print "\nOptimized!\n"

            # publish calibration state
            res = CameraCalibration()  ## initialize a new Message instance
            res.camera_pose = [camera.pose for camera in self.state.cameras]
            res.camera_id = [camera.camera_id for camera in self.state.cameras]
            res.time_stamp = [timestamp for timestamp in self.timestamps]  #copy
            res.m_count = len(res.time_stamp); #  self.measurement_count
            self.pub.publish(res)


def main():
    rospy.init_node('online_calibration')
    e = Estimator()
    rospy.spin()

if __name__ == '__main__':
    main()



