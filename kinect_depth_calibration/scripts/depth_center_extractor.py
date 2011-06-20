#!/usr/bin/python

import roslib; roslib.load_manifest('kinect_depth_calibration')
import rospy
from kinect_depth_calibration.srv import GetCheckerboardCenter, GetCheckerboardCenterResponse
from stereo_msgs.msg import DisparityImage
import cv
from cv_bridge import CvBridge, CvBridgeError
import threading


class DepthCenterExtractor:
    def __init__(self):
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.depth = None
        
        self.sub = rospy.Subscriber('disparity', DisparityImage, self.depth_cb)
        self.srv = rospy.Service('get_checkerboard_center', GetCheckerboardCenter, self.get_center_cb)


    def get_depth(self, disp, disp_msg):
        return disp_msg.f * disp_msg.T / disp


    def depth_cb(self, msg):
        with self.lock:
            self.depth = msg


    def get_center_cb(self, req):
        # get new disp message
        now = rospy.Time.now()
        while not rospy.is_shutdown():
            rospy.sleep(0.2)
            with self.lock:
                if self.depth and self.depth.header.stamp > now:
                    msg = self.depth
                    img = self.bridge.imgmsg_to_cv(self.depth.image, "passthrough")
                    break


        # get average disparity
        depth_sum = 0.0
        depth_nr = 0.0
        if req.min_x >= 0 and req.min_x < req.max_x and req.max_x < msg.image.width and req.min_y >= 0 and req.min_y < req.max_y and req.max_y < msg.image.height:
            for i in range(int(req.min_x), int(req.max_x)+1):
                for j in range(int(req.min_y), int(req.max_y)+1):
                    if img[j, i] > 0:
                        depth = self.get_depth(img[j, i], msg)
                        if depth > req.depth_prior*0.8 and depth < req.depth_prior*1.2:
                            depth_sum += depth
                            depth_nr += 1

        # check if we got at least 75% of points
        res = GetCheckerboardCenterResponse()
        if (req.max_x - req.min_x)*(req.max_y - req.min_y)*0.75 < depth_nr:
            res.depth = depth_sum / depth_nr
        else:
            res.depth = 0.0
            print "Could not find enough depth points: expected %f points but only found %f"%((req.max_x - req.min_x)*(req.max_y - req.min_y)*0.75, depth_nr)
            
        return res
    

        


def main():
    rospy.init_node('depth_center_extractor')
    rospy.sleep(0.1) # init sim time
    d = DepthCenterExtractor()
    rospy.spin()


if __name__ == '__main__':
    main()
