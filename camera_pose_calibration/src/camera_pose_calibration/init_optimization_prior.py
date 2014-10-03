import roslib; roslib.load_manifest('camera_pose_calibration')
import itertools
import numpy
import collections
import cv
import PyKDL
from tf_conversions import posemath


def get_target_pose(cam):
    # Populate object_points
    object_points = cv.fromarray(numpy.array([ [p.x, p.y, p.z ] for p in cam.features.object_points]))
    image_points = cv.fromarray(numpy.array([[p.x, p.y] for p in cam.features.image_points]))
    dist_coeffs = cv.fromarray(numpy.array([ [0.0, 0.0, 0.0, 0.0] ]))
    camera_matrix = numpy.array( [ [ cam.cam_info.P[0], cam.cam_info.P[1], cam.cam_info.P[2]  ],
                                   [ cam.cam_info.P[4], cam.cam_info.P[5], cam.cam_info.P[6]  ],
                                   [ cam.cam_info.P[8], cam.cam_info.P[9], cam.cam_info.P[10] ] ] )
    rot = cv.CreateMat(3, 1, cv.CV_32FC1)
    trans = cv.CreateMat(3, 1, cv.CV_32FC1)
    cv.FindExtrinsicCameraParams2(object_points, image_points, cv.fromarray(camera_matrix), dist_coeffs, rot, trans)
    # print "Rot: %f, %f, %f" % ( rot[0,0], rot[1,0], rot[2,0] )
    # print "Trans: %f, %f, %f" % ( trans[0,0], trans[1,0], trans[2,0] )
    rot3x3 = cv.CreateMat(3, 3, cv.CV_32FC1)
    cv.Rodrigues2(rot, rot3x3)
    frame = PyKDL.Frame()
    for i in range(3):
        frame.p[i] = trans[i,0]
    for i in range(3):
        for j in range(3):
            frame.M[i,j] = rot3x3[i,j]
    return frame

def read_observations(meas):
    # Stores the checkerboards observed by two cameras
    # camera_id -> camera_id -> [ (cb pose, cb pose, cb id) ]
    mutual_observations = collections.defaultdict(lambda: collections.defaultdict(list))
    
    checkerboard_id = 0
    for msg in meas:
        for M_cam1, M_cam2 in itertools.combinations(msg.M_cam, 2):
            cam1 = M_cam1.camera_id
            cam2 = M_cam2.camera_id
            p1 = get_target_pose(M_cam1)
            p2 = get_target_pose(M_cam2)

            mutual_observations[cam1][cam2].append( (p1, p2, checkerboard_id) )
            mutual_observations[cam2][cam1].append( (p2, p1, checkerboard_id) )

        checkerboard_id += 1
    return mutual_observations

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

def find_initial_poses(meas, root_cam = None):
    mutual_observations = read_observations(meas)

    if not root_cam:
        root_cam = mutual_observations.keys()[0]
    camera_poses = {}
    checkerboard_poses = {}
    bfs(root_cam, mutual_observations, camera_poses, checkerboard_poses)
    return camera_poses, checkerboard_poses
