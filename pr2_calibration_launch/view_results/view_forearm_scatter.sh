#! /bin/bash
rosrun pr2_calibration_estimation post_process.py /tmp/pr2_calibration/cal_measurements.bag /tmp/pr2_calibration/ `rospack find pr2_calibration_launch`/view_results/forearm_config.yaml
