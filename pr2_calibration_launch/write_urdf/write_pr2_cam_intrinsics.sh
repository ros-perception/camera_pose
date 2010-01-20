#! /bin/bash


if [ "$1" = ""]; then
    echo "Usage: ./write_cam_intrinsics [bagfile]"
else
    config=`rospack find pr2_calibration_launch`/results/system3.yaml

    echo "Bagfile: [$1]"
    echo "Config File: [$config]"
    echo "Flashing Wide Stereo Right"
    rosrun pr2_calibration_propagation propagate_baseline.py camera:=wide_stereo/right $1 $config wide_right_rect
    echo "**********"
    echo "Flashing Wide Stereo Left"
    rosrun pr2_calibration_propagation propagate_baseline.py camera:=wide_stereo/left $1 $config wide_left_rect
    echo "**********"
    echo "Flashing Narrow Stereo Right"
    rosrun pr2_calibration_propagation propagate_baseline.py camera:=narrow_stereo/right $1 $config narrow_right_rect
    echo "**********"
    echo "Flashing Narrow Stereo Left"
    rosrun pr2_calibration_propagation propagate_baseline.py camera:=narrow_stereo/left $1 $config narrow_left_rect
fi
