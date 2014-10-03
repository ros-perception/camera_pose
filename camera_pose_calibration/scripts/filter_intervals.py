#!/usr/bin/python



import rospy
import threading
from calibration_msgs.msg import Interval, CalibrationPattern



def diff(f1, f2):
    if not f1 or not f2:
        return 999999
    return pow(f1.x - f2.x, 2) + pow(f1.y - f2.y, 2)


class FilterIntervals:
    def __init__(self):
        self.min_duration = rospy.Duration(rospy.get_param('~min_duration', 0.5))
        self.min_motion = rospy.get_param('~min_motion', 5.0)
        self.lock = threading.Lock()
        self.feature = None
        self.last_feature = None

        self.pub = rospy.Publisher('interval_filtered', Interval)
        self.sub_intervals = rospy.Subscriber('interval', Interval, self.interval_cb)
        self.sub_features = rospy.Subscriber('features', CalibrationPattern, self.feature_cb)


    def interval_cb(self, msg):
        with self.lock:
            print "got interval!"
            duration = msg.end - msg.start
            if self.feature and diff(self.feature, self.last_feature) > self.min_motion and duration > self.min_duration:
                print "interval not good enough!"
                self.last_feature = self.feature
                self.pub.publish(msg)


    def feature_cb(self, msg):
        with self.lock:
            if len(msg.image_points) > 0:
                self.feature = msg.image_points[0]


def main():
    rospy.init_node('filter_intervals')
    f = FilterIntervals()
    rospy.spin()



if __name__ == '__main__':
    main()
