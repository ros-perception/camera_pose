#! /bin/bash


BAG_TIME=`stat --format=%z /tmp/pr2_calibration/cal_measurements.bag`
TARGET_FILENAME=/hwlog/cal_measurements_`date --date="$BAG_TIME" +%F-%H-%M-%S`.bag

cp /tmp/pr2_calibration/cal_measurements.bag $TARGET_FILENAME

echo "Copied measurements to: $TARGET_FILENAME"
