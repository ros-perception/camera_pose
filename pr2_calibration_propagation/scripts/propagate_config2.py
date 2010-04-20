#! /usr/bin/env python

import roslib; roslib.load_manifest('pr2_calibration_propagation')
import rospy
import pr2_calibration_propagation.update_urdf as update_urdf

import math
import pdb
import sys

try:
    import yaml
except ImportError, e:
    print >> sys.stderr, "Cannot import yaml. Please make sure the pyyaml system dependency is installed"
    raise e

if __name__ == '__main__':

    rospy.init_node("propagate_config")

    if len(rospy.myargv()) < 5:
        print "Usage: ./propagate_config [initial.yaml] [calibrated.yaml] [initial.xml] [cal_output.xml]"
        sys.exit(0)

    #filenames
    initial_yaml_filename    = rospy.myargv()[1]
    calibrated_yaml_filename = rospy.myargv()[2]
    robot_xml_filename       = rospy.myargv()[3]
    output_filename          = rospy.myargv()[4]

    #read in files
    system_default = yaml.load(file(initial_yaml_filename, 'r'))
    system_calibrated = yaml.load(file(calibrated_yaml_filename, 'r'))

    xml_out = update_urdf.update_urdf(system_default, system_calibrated, open(robot_xml_filename).read())

    outfile = open(output_filename, 'w')
    outfile.write(xml_out)
    outfile.close()
