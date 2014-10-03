#!/usr/bin/python

from camera_pose_calibration.camera_info_converter import CameraInfoConverter

def main():
    rospy.init_node('camera_info_converter')
    camera_converter = CameraInfoConverter()
    rospy.spin()



if __name__ == '__main__':
    main()
