#! /usr/bin/env python

BAG = '/u/vpradeep/kinect_bags/kinect_extrinsics_2011-04-05-16-01-28.bag'

import sys, time, optparse
import itertools
import collections

import roslib
roslib.load_manifest('megacal_estimation')
import PyKDL
from calibration_msgs.msg import *

import rosbag

def pose_msg_to_kdl(m):
    return PyKDL.Frame(PyKDL.Rotation.Quaternion(m.orientation.x, m.orientation.y,
                                                 m.orientation.z, m.orientation.w),
                       PyKDL.Vector(m.position.x, m.position.y, m.position.z))


# Stores the checkerboards observed by two cameras
# camera_id -> camera_id -> [ (cb pose, cb pose, cb id) ]
mutual_observations = collections.defaultdict(lambda: collections.defaultdict(list))

bag = rosbag.Bag(BAG)
checkerboard_id = 0
for topic, msg, t in bag:
    assert topic == 'robot_measurement'
    for M_cam1, M_cam2 in itertools.combinations(msg.M_cam, 2):
        cam1 = M_cam1.camera_id
        cam2 = M_cam2.camera_id
        p1 = pose_msg_to_kdl(M_cam1.features.object_pose.pose)
        p2 = pose_msg_to_kdl(M_cam2.features.object_pose.pose)
        
        mutual_observations[cam1][cam2].append( (p1, p2, checkerboard_id) )
        mutual_observations[cam2][cam1].append( (p2, p1, checkerboard_id) )

    checkerboard_id += 1

bag.close()

# Populates cameras_seen and checkerboards_seen
# root_cam: camera id to start from
# observations: camera_id -> camera_id -> [ (cb pose, cb pose, checkerbord id) ]
# cameras_seen: camera_id -> pose
# checkerboards_seen: checkerboard_id -> pose
def bfs(root_cam, observations, cameras_seen, checkerboards_seen):
    q = [root_cam]
    cameras_seen[root_cam] = PyKDL.Frame()
    while q:
        cam1, q = q[0], q[1:]  # pop
        cam1_pose = cameras_seen[cam1]

        for cam2, checkerboards in observations[cam1].iteritems():
            assert checkerboards
            if cam2 not in cameras_seen:
                q.append(cam2)
                
                # Cam2Pose = Cam1Pose * Cam1->CB * CB->Cam2
                cameras_seen[cam2] = cam1_pose * checkerboards[0][0] * checkerboards[0][1].Inverse()

            for cb in checkerboards:
                if cb[2] not in checkerboards_seen:
                    checkerboards_seen[cb[2]] = cam1_pose * cb[0]
    

root_cam = mutual_observations.keys()[0]
camera_poses = {}
checkerboard_poses = {}
bfs(root_cam, mutual_observations, camera_poses, checkerboard_poses)

print camera_poses
print "\n\n"
print checkerboard_poses
