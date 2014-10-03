# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(camera_pose_calibration)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS geometry_msgs calibration_msgs sensor_msgs image_cb_detector monocam_settler interval_intersection joint_states_settler tf_conversions tf2_ros tf2_kdl message_filters)

# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
# CATKIN_MIGRATION: removed during catkin migration
# cmake_minimum_required(VERSION 2.4.6)

# CATKIN_MIGRATION: removed during catkin migration
# include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)


# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_init()

#uncomment if you have defined messages
add_message_files(
  FILES
  CalibrationEstimate.msg
  CameraCalibration.msg
  CameraMeasurement.msg
  CameraPose.msg
  RobotMeasurement.msg
)
#uncomment if you have defined services
add_service_files(
  FILES
  TriggerOne.srv
)

#common commands for building c++ executables and libraries
#add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#
# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_boost_directories()
#find_package(Boost REQUIRED COMPONENTS thread)
#include_directories(${Boost_INCLUDE_DIRS})
#target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
#add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

#download_test_data(http://pr.willowgarage.com/data/camera_pose_calibration/camera_pose_cal_regression_data.bag test/camera_pose_cal_regression_data.bag fcd96a5c7a74a064640753fd9d82fff3)
#add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_camera_pose_calibration.launch)

catkin_python_setup()

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES calibration_msgs geometry_msgs sensor_msgs std_msgs
)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS geometry_msgs calibration_msgs sensor_msgs image_cb_detector monocam_settler interval_intersection joint_states_settler kdl tf_conversions tf2_ros tf2_kdl message_filters opencv2
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)
