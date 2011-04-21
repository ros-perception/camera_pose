
import yaml
from camera_pose_calibration.msg import CameraPose

from tf_conversions import posemath

def to_urdf(cameras):
    parent_link = "world_link"
    urdf = ""
    urdf += '<?xml version="1.0">\n'
    urdf += '<robot>\n'
    urdf += '  <link name="%s"/>\n' % parent_link
    for cur_cam in cameras:
        urdf += '  <link name="%s">' % cur_cam.camera_id
        urdf += '  <joint name="%s_joint" type="fixed">\n' % cur_cam.camera_id
        urdf += '    <parent link="%s"/>\n' % parent_link
        urdf += '    <child  link="%s"/>\n' % cur_cam.camera_id

        f = posemath.fromMsg(cur_cam.pose)
        urdf += '    <origin xyz="%f %f %f" rpy="%f %f %f" />\n' % ( (f.p.x(), f.p.y(), f.p.z()) + f.M.GetRPY() )
        urdf += '  </joint>\n'
    urdf += '</robot>\n'
    return urdf

def to_dict_list(cameras):
    d = [ {'camera_id': cam.camera_id,
           'position':
             {'x':cam.pose.position.x, 'y':cam.pose.position.y, 'z':cam.pose.position.z},
           'orientation':
             {'x':float(cam.pose.orientation.x), 'y':float(cam.pose.orientation.y), 'z':float(cam.pose.orientation.z), 'w':float(cam.pose.orientation.w)}
           } for cam in cameras]
    return d

