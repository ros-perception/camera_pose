#! /bin/bash

roslaunch pr2_calibration_launch estimate_pr2_beta.launch
rosrun pr2_calibration_launch write_pr2_beta.sh
rosrun pr2_calibration_launch backup_measurements.sh
