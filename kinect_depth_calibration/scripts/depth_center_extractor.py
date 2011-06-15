#!/usr/bin/python

import roslib; roslib.load_manifest('kinect_depth_calibration')
import rospy
from kinect_depth_calibration.srv import GetCheckerboardCenter, GetCheckerboardCenterResponse
from stereo_msgs.msg import DisparityImage
import cv
import threading


class DepthCenterExtractor:
    def __init__(self):
        self.lock = threading.Lock()
        self.depth = None
        
        self.sub = rospy.Subscriber('disparity', DisparityImage, self.depth_cb)
        self.srv = rospy.Service('get_checkerbaord_center', GetCheckerboardCenter, self.get_center_cb)


    def get_depth(self, disp, disp_msg):
        return disp_msg.f/(disp_msg.T*disp)


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
                    break


        # get average disparity
        depth_sum = 0.0
        depth_nr = 0.0
        if req.min_x >= 0 and req.min_x < req.max_x and req.max_x < msg.image.width and req.min_y >= 0 and req.min_y < req.max_y and req.max_y < msg.image.height:
            for i in range(req.min_x, req.max_x+1):
                for j in range(req.min_y, req.may_y+1):
                    depth = self.get_depth(msg.image[j, i], msg)
                    if depth > req.depth*0.8 and depth < req.depth*1.2:
                        depth_sum += depth
                        depth_nr += 1

        res = GetCheckerboardCenterResponse()

        # check if we got at least 75% of points
        if (req.max_x - req.min_x)*(req.max_y - req.min_y)*0.75 < depth_nr:
            res.depth = depth_sum / depth_nr
        else:
            res.depth = 0.0
            print "Could not find enough depth points"
            
        return res
    

        


def main():
    rospy.init_node('depth_center_extractor')
    rospy.sleep(0.1) # init sim time
    d = DepthCenterExtractor()
    rospy.spin()


if __name__ == '__main__':
    main()
