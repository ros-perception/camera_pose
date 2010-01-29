#! /bin/bash

original_system=`rospack find pr2_calibration_launch`/estimate_params/config_pr2_beta/system.yaml
final_system=/tmp/pr2_calibration/system4.yaml
urdf_in=/etc/ros/urdf/robot_uncalibrated.xml
urdf_out=robot_calibrated.xml
`rospack find pr2_calibration_propagation`/scripts/propagate_config_ref_positions.py $original_system $final_system $urdf_in $urdf_out
echo "Wrote new URDF to $urdf_out"