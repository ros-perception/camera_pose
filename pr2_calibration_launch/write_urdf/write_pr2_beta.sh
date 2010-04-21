#! /bin/bash

original_system=`rospack find pr2_calibration_launch`/estimate_params/config_pr2_beta/system.yaml
final_system=/tmp/pr2_calibration/system_calibrated.yaml
bag_in=/tmp/pr2_calibration/cal_measurements.bag
urdf_out=robot_calibrated.xml
`rospack find pr2_calibration_propagation`/scripts/propagate_config2.py $original_system $final_system $bag_in $urdf_out > /tmp/pr2_calibration/urdf_writer_debug.log
if [ $? = 0 ]
then
    echo "Wrote new URDF to $urdf_out"
else
    echo "Error writing URDF"
fi

